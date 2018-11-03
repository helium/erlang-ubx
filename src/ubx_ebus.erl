-module('ubx_ebus').

-behavior(ebus_object).

-define(OBJECT_PATH, "/com/helium/GPS").
-define(OBJECT_INTERFACE, "com.helium.GPS").

%% ebus_object
-export([start/4, start_link/4, init/1, handle_info/2, handle_message/3]).

-record(state, {
                ubx_handle :: pid(),
                gps_lock=false :: boolean(),
                gps_position=#{} :: #{string() => float()}
               }).

start(Bus, Type, Device, Options) ->
    ebus_object:start(Bus, ?OBJECT_PATH, ?MODULE, [Type, Device, Options], []).

start_link(Bus, Type, Device, Options) ->
    ebus_object:start_link(Bus, ?OBJECT_PATH, ?MODULE, [Type, Device, Options], []).

-ifndef(TEST).
init([Type, Device, Options]) ->
    {ok, Pid} = ubx:start_link(Type, Device, Options, self()),
    ubx:enable_message(nav_posllh, 5, Pid),
    ubx:enable_message(nav_sol, 5, Pid),
    {ok, #state{ubx_handle=Pid}}.
-else.
init([_Type, _Device, _Options]) ->
    {ok, #state{ubx_handle=test}}.
-endif.

handle_message("Position", _Msg, State=#state{}) ->
    {reply,
     [bool, {dict, string, double}],
     [State#state.gps_lock, State#state.gps_position], State};
handle_message(Member, _Msg, State) ->
    lager:warning("Unhandled message~p", Member),
    {noreply, State}.

handle_info({nav_sol, GPSFix}, State) ->
    case GPSFix == 3 of
        true ->
            {noreply, State#state{gps_lock=true}};
        false ->
            {noreply, State#state{gps_lock=false}}
    end;
handle_info({nav_posllh, _}, State=#state{gps_lock=false}) ->
    {noreply, State};
handle_info({nav_posllh, {Lat,Lon,Height,HorizontalAcc,VerticalAcc}}, State=#state{}) ->
    Position = #{
                 "lat" => Lat,
                 "lon" => Lon,
                 "height" => Height,
                 "h_accuracy" => HorizontalAcc,
                 "v_accuracy" => VerticalAcc
                },
    {noreply, State#state{gps_position=Position},
     {signal, ?OBJECT_INTERFACE, "Position",
      [{dict, string, double}], [Position]}};

handle_info(_Msg, State) ->
    lager:warning("unhandled info message ~p", [_Msg]),
    {noreply, State}.
