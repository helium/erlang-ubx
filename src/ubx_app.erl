-module(ubx_app).

-behavior(application).

-export([start/2, stop/1]).

start(_StartType, _StartArgs) ->
    application:ensure_all_started(ebus),
    {ok, Bus} = ebus:system(),
    ubx_sup:start_link([Bus]).

stop(_State) ->
    ok.
