-module(adf).

-export([program/0]).

program() ->
    {ok, SPI} = spi:start_link("spidev2.2", []),
    {ok, Gpio} = gpio:start_link(84, output),
    gpio:write(Gpio, 0),
    % Initialization Latch
    spi:transfer(SPI, <<16#1F, 16#80, 16#93>>),
    timer:sleep(100),
    gpio:write(Gpio, 1),
    timer:sleep(100),
    gpio:write(Gpio, 0),
    % Function Latch
    spi:transfer(SPI, <<16#1F, 16#80, 16#92>>),
    timer:sleep(100),
    gpio:write(Gpio, 1),
    timer:sleep(100),
    gpio:write(Gpio, 0),
    % Reference Counter Latch
    spi:transfer(SPI, <<16#00, 16#00, 16#04>>),
    timer:sleep(100),
    gpio:write(Gpio, 1),
    timer:sleep(100),
    gpio:write(Gpio, 0),
    % N Counter Latch
    %spi:transfer(SPI, <<16#00, 16#01, 16#01>>),
    spi:transfer(SPI, <<16#00, 16#05, 16#01>>),
    timer:sleep(100),
    gpio:write(Gpio, 1),
    timer:sleep(100),
    gpio:write(Gpio, 0),
    ok.
