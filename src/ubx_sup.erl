-module(ubx_sup).

-behaviour(supervisor).

%% supervisor
-export([init/1, start_link/1]).

start_link([Bus]) ->
    supervisor:start_link({local, ?MODULE}, ?MODULE, [Bus]).

init([Bus]) ->
    SupFlags = #{strategy => one_for_all,
                 intensity => 1,
                 period => 5},
    ChildSpecs = [
                  #{id => ubx_ebus,
                    start => {ubx_ebus, start_link, [Bus, spi, "spidev1.0", []]},
                    restart => permanent,
                    type => worker}
                 ],

    {ok, {SupFlags, ChildSpecs}}.
