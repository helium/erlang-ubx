-module(ebus_SUITE).

-include_lib("common_test/include/ct.hrl").
-include_lib("eunit/include/eunit.hrl").

-export([all/0, init_per_suite/1, end_per_suite/1,
         init_per_testcase/2, end_per_testcase/2]).
-export([message_test/1, signal_test/1]).

all() ->
    [ message_test,
      signal_test
    ].

init_per_suite(Config) ->
    application:set_env(lager, error_logger_flush_queue, false),
    application:ensure_all_started(lager),
    lager:set_loglevel(lager_console_backend, debug),

    {ok, B} = ebus:start(),
    ?assertEqual(ok, ebus:request_name(B, "com.helium.test",
                                       [{replace_existing, true}])),
    [{bus, B}| Config].

end_per_suite(Config) ->
    B = ?config(bus, Config),
    ebus:stop(B, normal).

init_per_testcase(_, Config) ->
    B = ?config(bus, Config),
    {ok, Obj} = ubx_ebus:start(B, type, device, options),
    [{ubx_object, Obj} | Config].

end_per_testcase(_, Config) ->
    Obj = ?config(ubx_object, Config),
    ebus_object:stop(Obj, normal).

message_test(Config) ->
    B = ?config(bus, Config),
    Obj = ?config(ubx_object, Config),

    {ok, Proxy} = ebus_proxy:start(B, "com.helium.test", "/com/helium/GPS", []),

    ?assertEqual({ok, [false, #{}]}, ebus_proxy:call(Proxy, "Position")),

    %% Invalid gps fix leaves lock as false
    Obj ! {nav_sol, 1},
    ?assertEqual({ok, [false, #{}]}, ebus_proxy:call(Proxy, "Position")),
    %% Receiving a ubx reading while not gps locked does not update Position
    Obj ! {nav_posllh,{0,0,0,0,0}},
    ?assertEqual({ok, [false, #{}]}, ebus_proxy:call(Proxy, "Position")),
    %% Provide a gps fix and position to the service
    Obj ! {nav_sol, 3},
    ?assertEqual({ok, [true, #{}]}, ebus_proxy:call(Proxy, "Position")),
    Obj ! {nav_posllh,{37.7705072,-122.4191044,40989,21974,3074}},
    %% Note that integer values are converted to doubles in the map
    Pos = #{
            "lat" => 37.7705072,
            "lon" => -122.4191044,
            "height" => 40989.0,
            "h_accuracy" => 21974.0,
            "v_accuracy" => 3074.0
           },
    ?assertEqual({ok, [true, Pos]}, ebus_proxy:call(Proxy, "Position")),

    ok.

signal_test(Config) ->
    B = ?config(bus, Config),
    Obj = ?config(ubx_object, Config),

    %% Add a match and filter for signals
    ok = ebus:add_match(B, "type=signal"),
    {ok, Filter} = ebus:add_filter(B, self(),
                                   #{
                                     path => "/com/helium/GPS"
                                    }),


    %% Provide a gps fix and position to the service
    Obj ! {nav_sol, 3},
    Obj ! {nav_posllh,{37.7705072,-122.4191044,40989,21974,3074}},

    %% Receive the signal
    Msg = receive
              {filter_match, M2} -> M2
          after 5000 -> erlang:exit(timeout_filter)
          end,

    Pos = #{
            "lat" => 37.7705072,
            "lon" => -122.4191044,
            "height" => 40989.0,
            "h_accuracy" => 21974.0,
            "v_accuracy" => 3074.0
           },
    ?assertEqual({ok, [Pos]}, ebus_message:args(Msg)),

    ok = ebus:remove_filter(B, Filter),
    ok.
