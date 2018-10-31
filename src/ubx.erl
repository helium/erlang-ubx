-module(ubx).
-behaviour(gen_server).

-define(U1, 8/integer-unsigned). %% unsigned char
-define(I1, 8/integer-signed). %% signed char
-define(X1, 8/integer). %% bitfield
-define(U2, 16/integer-unsigned-little). %% unsigned short
-define(I2, 16/integer-signed-little). %% signed short
-define(X2, 16/integer-little). %% bitfields are little endian
-define(U4, 32/integer-unsigned-little). %% unsigned long
-define(I4, 32/integer-signed-little). %% signed long
-define(X4, 32/integer-little). %% bitfields are little endian with big endian fields
-define(R4, 32/float-little). %% IEEE 754 single precision (endianness??)
-define(R8, 64/float-little). %% IEEE 754 double precision (endianness??)

-define(HEADER1, 16#b5).
-define(HEADER2, 16#62).

-define(MM_TO_FEET, 0.00328084).
-define(MM_TO_MILES, 0.0000006213712).

%% message classes
-define(MON, 16#0a).
-define(CFG, 16#06).
-define(TIM, 16#0d).
-define(NAV, 16#01).

%% message IDs
-define(TM2, 16#03).
-define(MSG, 16#01).
-define(VER, 16#04).
-define(PVT, 16#07).
-define(HW, 16#09).
-define(SAT, 16#35).
-define(PRT, 16#00).
-define(STATUS, 16#03).
-define(POSLLH, 16#02).
-define(SOL, 16#06).
-define(CFG2, 16#09).
-define(TP5, 16#31).
-define(ANT, 16#13).

-record(state, {
          device :: port() | pid(),
          devicetype :: spi | serial,
          buffer = <<>> :: binary(),
          controlling_process :: pid(),
          ack,
          poll,
          gpio
         }).

-export([start_link/4, enable_message/3, disable_message/2, poll_message/2, poll_message/3, parse/1]).

-export([init/1, handle_info/2, handle_cast/2, handle_call/3]).

start_link(Type, Filename, Options, ControllingProcess) ->
    gen_server:start_link(?MODULE, [Type, Filename, Options, ControllingProcess], []).

enable_message(Msg, Frequency, Pid) ->
    {MsgClass, MsgID} = resolve(Msg),
    gen_server:call(Pid, {enable_message, MsgClass, MsgID, Frequency}).

disable_message(Msg, Pid) ->
    {MsgClass, MsgID} = resolve(Msg),
    gen_server:call(Pid, {enable_message, MsgClass, MsgID, 0}).

poll_message(Msg, Pid) ->
    {MsgClass, MsgID} = resolve(Msg),
    gen_server:call(Pid, {poll_message, MsgClass, MsgID, <<>>}).

poll_message(Msg, Payload, Pid) ->
    {MsgClass, MsgID} = resolve(Msg),
    gen_server:call(Pid, {poll_message, MsgClass, MsgID, Payload}).


init([spi, Filename, Options, ControllingProcess]) ->
    {ok, Spi} = spi:start_link(Filename, Options),
    {ok, Gpio} = gpio:start_link(69, input),
    State0 = #state{device=Spi, devicetype=spi, controlling_process=ControllingProcess, gpio=Gpio},
    gpio:register_int(Gpio),
    gpio:set_int(Gpio, rising),
    %% disable LNA_EN pin function so we can repurpose it as TX_READY
    send(State0, frame(?CFG, ?ANT, <<0, 0, 16#f0, 16#39>>)),
    {State, {ack, ?CFG, ?ANT}} = get_ack(State0),
    Port = 4,
    Reserved1 = <<0>>,
    BytesWaiting = 1,
    PIO = 16, %% LNA_EN on NEO-M8T
    Polarity = 0, %% active high
    Active = 1,
    <<TXReadyInt:16/integer-unsigned-big>> = <<BytesWaiting:9/integer-unsigned-big, PIO:5/integer-unsigned-big, Polarity:1/integer, Active:1/integer>>,
    %% So apparently it's hard to pack the bits into the right order because the bitfield is big endian (MSB) but the byte order is little endian
    %TXReady = <<185, 0>>,
    TXReady = <<TXReadyInt:16/integer-unsigned-little>>,
    Mode = 0,
    Reserved2 = <<0, 0, 0, 0>>,
    InProtoMask = 1, %% only UBX
    OutProtoMask = 1, %% only UBX
    Flags = 0,
    Reserved3 = <<0, 0>>,
    send(State, frame(?CFG, ?PRT, <<Port:?U1, Reserved1/binary, TXReady/binary, Mode:?X4, Reserved2/binary, InProtoMask:?X2, OutProtoMask:?X2, Flags:?X2, Reserved3/binary>>)),
    {NewState, {ack, ?CFG, ?PRT}} = get_ack(State),
    TPIdx = 0,
    Version = 1,
    Reserved = <<0, 0>>,
    AntennaCableDelay = 0, %% XXX in nanoseconds
    RFGroupDelay = 0, %% read only?
    FreqPeriod = 1, %% 1mHz
    PulseLenRatio = 0, %% disable the pulse when not locked
    FreqPeriodLock = 8000000, %% 4mHz
    PulseLenRatioLock = 2147483648, %% 50% duty cycle when locked
    UserConfigDelay = 0,
    TPActive = 1,
    LockGNSSFreq = 1,
    LockedOtherSet = 1,
    IsFreq = 1,
    IsLength = 0,
    AlignToToW = 1,
    ClockPolarity = 1,
    GridUTCGNSS = 0,
    SyncMode = 1, %% Only use the locked mode while we have a lock
    <<TPFlags:32/integer-unsigned-big>> = <<0:18/integer, SyncMode:3/integer-unsigned-big, GridUTCGNSS:4/integer-unsigned-big, ClockPolarity:1/integer, AlignToToW:1/integer, IsLength:1/integer, IsFreq:1/integer, LockedOtherSet:1/integer, LockGNSSFreq:1/integer, TPActive:1/integer>>,
    send(NewState, frame(?CFG, ?TP5, <<TPIdx:?U1, Version:?U1, Reserved/binary, AntennaCableDelay:?I2, RFGroupDelay:?I2, FreqPeriod:?U4, FreqPeriodLock:?U4, PulseLenRatio:?U4, PulseLenRatioLock:?U4, UserConfigDelay:?I4, TPFlags:?X4>>)),

    {NewState2, {ack, ?CFG, ?TP5}} = get_ack(State),
    %% PIO changes don't take effect until a config save
    send(NewState2, frame(?CFG, ?CFG2, <<0:?X4, 16#ff, 16#ff, 0, 0, 0:?X4, 23:?X1>>)),
    {NewState3, {ack, ?CFG, ?CFG2}} = get_ack(NewState2),
    case gpio:read(Gpio) of
        1 ->
            self() ! {gpio_interrupt,69,rising};
        0 ->
            ok
    end,
    {ok, NewState3};

init([serial, Filename, Options, ControllingProcess]) ->
    SerialPort = serial:start([{open, Filename} | Options]),
    State = #state{device=SerialPort, devicetype=serial, controlling_process=ControllingProcess},
    TXReady = 0,
    Port=3,
    Reserved2 = <<0, 0, 0, 0, 0, 0, 0, 0>>,
    InProtoMask = 1, %% only UBX
    OutProtoMask = 1, %% only UBX
    Reserved3 = <<0, 0>>,
    Reserved4 = <<0, 0>>,
    send(State, frame(?CFG, ?PRT, <<Port:?U1, 0:?U1, TXReady:?X2, Reserved2/binary, InProtoMask:?X2, OutProtoMask:?X2, Reserved3/binary, Reserved4/binary>>)),
    {NewState, {ack, ?CFG, ?PRT}} = get_ack(State),
    {ok, NewState}.

handle_info({gpio_interrupt,69,rising}, State = #state{ack=Ack, poll=Poll}) ->
    %io:format("handling interrupt~n"),
    {ok, NewerState} = case get_packet(State) of
        {NewState, {error, Error}} ->
                               io:format("error ~p~n", [Error]),
                        {ok, NewState};
        {NewState, {ack, ?CFG, ?MSG}} ->
                        io:format("ack~n"),
            case Ack of
                {From, Ref} ->
                    erlang:cancel_timer(Ref),
                    gen_server:reply(From, ok),
                    {ok, NewState#state{ack=undefined}};
                undefined ->
                    {ok, NewState}
            end;
        {NewState, {nack, ?CFG, ?MSG}} ->
                        io:format("nack~n"),
            case Ack of
                {From, Ref} ->
                    erlang:cancel_timer(Ref),
                    gen_server:reply(From, {error, nack}),
                    NewState#state{ack=undefined};
                undefined ->
                    {ok, NewState}
            end;
        {NewState, Packet = {Msg, _}} ->
                        %io:format("packet~n"),
            case Poll of
                {Msg, From, Ref} ->
                    erlang:cancel_timer(Ref),
                    gen_server:reply(From, {ok, Packet}),
                    {ok, NewState#state{poll=undefined}};
                _ ->
                    NewState#state.controlling_process ! Packet,
                    {ok, NewState}
            end
    end,
    case gpio:read(NewerState#state.gpio) of
        1 ->
            %io:format("interrupt still high~n"),
            handle_info({gpio_interrupt, 69, rising}, NewerState);
        0 ->
            %io:format("interrupt went low~n"),
            {noreply, NewerState}
    end;
handle_info({data, Bytes}, State) ->
    case get_packet(State#state{buffer= <<(State#state.buffer)/binary, Bytes/binary>>}) of
        {NewState, {error, _}} ->
            {noreply, NewState};
        {NewState, Packet} ->
            State#state.controlling_process ! Packet,
            {noreply, NewState}
    end;
handle_info(ack_timeout, State = #state{ack={From, _Ref}}) ->
    gen_server:reply(From, {error, timeout}),
    {noreply, State#state{ack=undefined}};
handle_info(poll_timeout, State = #state{poll={_Msg, From, _Ref}}) ->
    gen_server:reply(From, {error, timeout}),
    {noreply, State#state{poll=undefined}};
handle_info(_Msg, State) ->
    {noreply, State}.


handle_cast(_Msg, State) ->
    {noreply, State}.

handle_call({enable_message, MsgClass, MsgID, Frequency}, From, State) ->
    case State#state.ack of
        undefined ->
            io:format("hello~n"),
            send(State, frame(?CFG, ?MSG, <<MsgClass, MsgID, Frequency>>)),
            {noreply, register_ack(From, State)};
        _ ->
            {reply, {error, busy}, State}
    end;
handle_call({poll_message, MsgClass, MsgID, Payload}, From, State) ->
    case State#state.poll of
        undefined ->
            Msg = resolve(MsgClass, MsgID),
            send(State, frame(MsgClass, MsgID, Payload)),
            {noreply, register_poll(Msg, From, State)};
        _ ->
            {noreply, {error, busy}, State}
    end;
handle_call(Msg, _From, State) ->
    {reply, {unknown_call, Msg}, State}.

send(#state{devicetype=spi, device=Device}, Packet) ->
    io:format("Sending~s~n", [lists:flatten([ io_lib:format(" ~.16b", [X]) || <<X:8/integer>> <= Packet ])]),
    spi:transfer(Device, Packet);
send(#state{devicetype=serial, device=Device}, Packet) ->
    Device ! {send, Packet}.

recv(State=#state{devicetype=spi, device=Device}, Length) ->
    %io:format("SPI read ~p~n", [Length]),
    {State, spi:transfer(Device, binary:copy(<<255>>, Length))};
recv(State=#state{devicetype=serial, buffer=Buffer}, Length) ->
    case byte_size(Buffer) < Length of
        true ->
            receive
                {data, Bytes} ->
                    case byte_size(Bytes) + byte_size(Buffer) >= Length of
                        true ->
                            <<Result:Length/binary, NewBuffer/binary>> = <<Buffer/binary, Bytes/binary>>,
                            {State#state{buffer=NewBuffer}, Result};
                        false ->
                            recv(State#state{buffer = <<Buffer/binary, Bytes/binary>>}, Length)
                    end
            after
                5000 ->
                    {State, {error, timeout}}
            end;
        false ->
            <<Result:Length/binary, NewBuffer/binary>> = Buffer,
            {State#state{buffer=NewBuffer}, Result}
    end.

is_spi(#state{devicetype=spi}) -> true;
is_spi(_) -> false.

frame(Class, Id, Payload) ->
    {CKA, CKB} = checksum(<<Class, Id, (byte_size(Payload)):?U2, Payload/binary>>),
    <<?HEADER1, ?HEADER2, Class, Id, (byte_size(Payload)):?U2, Payload/binary, CKA, CKB>>.

get_ack(State) ->
    case get_packet(State, <<>>, 10000) of
        {NewState, {ack, _, _} = Ack} ->
            {NewState, Ack};
        {NewState, {nack, _, _} = Nack} ->
            {NewState, Nack};
        {NewState, {error, _} = Error} ->
            {NewState, Error};
        {NewState, Other} ->
            io:format("other packet ~p~n", [Other]),
            NewState#state.controlling_process ! {packet, Other},
            get_ack(NewState)
    end.

get_packet(State) ->
    get_packet(State, <<>>, 6).

get_packet(State, _, 0) ->
    {State, {error, timeout}};
get_packet(State, Acc, Count) ->
    %io:format("reading ~p bytes~n", [6 - byte_size(Acc)]),
    {NewState, Reply} = recv(State, 6 - byte_size(Acc)),
    case Reply of
        {error, _} ->
            {NewState, Reply};
        _ ->
            case list_to_binary([Acc, Reply]) of
                <<255, 255, 255, 255, 255, 255>> ->
                    get_packet(NewState, Acc, Count - 1);
                <<_:8/integer, _:8/integer, _:8/integer, _:8/integer, _:8/integer, ?HEADER1>> ->
                    get_packet(NewState, <<?HEADER1>>, Count - 1);
                <<_:8/integer, _:8/integer, _:8/integer, _:8/integer, ?HEADER1, ?HEADER2>> ->
                    get_packet(NewState, <<?HEADER1, ?HEADER2>>, Count - 1);
                <<_:8/integer, _:8/integer, _:8/integer, ?HEADER1, ?HEADER2, Class:?U1>> ->
                    get_packet(NewState, <<?HEADER1, ?HEADER2, Class:?U1>>, Count - 1);
                <<_:8/integer, _:8/integer, ?HEADER1, ?HEADER2, Class:?U1, ID:?U1>> ->
                    get_packet(NewState, <<?HEADER1, ?HEADER2, Class:?U1, ID:?U1>>, Count - 1);
                <<_:8/integer, ?HEADER1, ?HEADER2, Class:?U1, ID:?U1, L1>> ->
                    get_packet(NewState, <<?HEADER1, ?HEADER2, Class:?U1, ID:?U1, L1>>, Count - 1);
                <<?HEADER1, ?HEADER2, _Class:?U1, _ID:?U1, Length:?U2>> = Header ->
                    {NewState2, Body} = case is_spi(State) andalso Length + 2 > 255 of
                                            true ->
                                                {_, B1} = recv(NewState, 128),
                                                {_, B2} = recv(NewState, (Length + 2) rem 128),
                                                {NewState, <<B1/binary, B2/binary>>};
                                            false ->
                                                recv(NewState, Length + 2)
                                        end,
                    {NewState2, parse(<<Header/binary, Body/binary>>)};
                _Other ->
                    %{NewState, {error, {goop, Other}}}
                    get_packet(NewState, <<>>, Count - 1)
            end
    end.

parse(<<?HEADER1, ?HEADER2, Class:?U1, ID:?U1, Length:?U2, Body:Length/binary, CK_A, CK_B>>) ->
    case checksum(<<Class:?U1, ID:?U1, Length:?U2, Body/binary>>) of
        {CK_A, CK_B} ->
            %% checksum is OK
            parse(Class, ID, Body);
        _ ->
            {error, bad_checksum}
    end.

%% UBX-ACK-ACK
parse(16#5, 16#1, <<ClassID:?U1, MsgID:?U1>>) ->
    {ack, ClassID, MsgID};

%% UBX-ACK-NACK
parse(16#5, 16#0, <<ClassID:?U1, MsgID:?U1>>) ->
    {nack, ClassID, MsgID};

%% UBX-NAV-PVT
parse(?NAV, ?PVT, <<_ITOW:?U4, _Year:?U2, _Month:?U1, _Day:?U1, _Hour:?U1, _Min:?U1, _Sec:?U1,
                    _Valid:?X1, TimeAccuracy:?U4, _Nano:?I4, _FixType:?U1, _Flags:?X1, _Flags2:?X1,
                    _NumSV:?U1, Longitude:?I4, Latitude:?I4, _Height:?I4, HeightMSL:?I4, HorizontalAccuracy:?U4,
                    VerticalAccuracy:?U4, _VelocityN:?I4, _VelocityE:?I4, _VelocityD:?I4, _Speed:?I4,
                    _Heading:?I4, _SpeedAccuracy:?U4, _HeadingAccuracy:?U4, _PositionDOP:?U2, _:6/binary,
                    _VehicleHeading:?I4, _MagneticDeclination:?I2, _MagneticAccuracy:?U2>>) ->
    %io:format("ITOW ~p, Year ~p, Month ~p, Day ~p, Hour ~p, Minute ~p, Second ~p~n",
    %[ITOW, Year, Month, Day, Hour, Min, Sec]),
    %io:format("Valid ~p, TimeAccuracy ~p, Nano ~p, FixTime ~p, Flags ~p, Flags2 ~p~n",
    %[Valid, TimeAccuracy, Nano, FixType, Flags, Flags2]),
    %io:format("NumSatellites ~p, Longitude ~f, Latitude ~f, Height Ellipsoid ~p ft, Height MeanSeaLevel ~f ft, Horizontal Accuracy ~f ft~n",
    %[NumSV, Longitude * 1.0e-7, Latitude * 1.0e-7, Height * ?MM_TO_FEET, HeightMSL * ?MM_TO_FEET, HorizontalAccuracy * ?MM_TO_FEET]),
    %io:format("Vertical Accuracy ~f ft, Velocity North ~f mph, Velocity East ~f mph, Velocity Down ~f mph, Ground Speed ~f mph~n",
    %[VerticalAccuracy * ?MM_TO_FEET, VelocityN * ?MM_TO_MILES, VelocityE * ?MM_TO_MILES, VelocityD * ?MM_TO_MILES, Speed * ?MM_TO_MILES]),
    {nav_pvt, {Latitude * 1.0e-7, Longitude * 1.0e-7, HeightMSL, HorizontalAccuracy, VerticalAccuracy, TimeAccuracy}};
%% UBX-NAV-POSLLH
parse(?NAV, ?POSLLH, <<_ITOW:?U4, Longitude:?I4, Latitude:?I4, _Height:?I4, HeightMSL:?I4, HorizontalAccuracy:?U4, VerticalAccuracy:?U4>>) ->
    %io:format("Longitude ~f, Latitude ~f, Height Ellipsoid ~p ft, Height MeanSeaLevel ~f ft, Horizontal Accuracy ~f ft, Vertical Accuracy ~f~n",
    %[Longitude * 1.0e-7, Latitude * 1.0e-7, Height * ?MM_TO_FEET, HeightMSL * ?MM_TO_FEET, HorizontalAccuracy * ?MM_TO_FEET, VerticalAccuracy * ?MM_TO_FEET]),
    {nav_posllh, {Latitude * 1.0e-7, Longitude * 1.0e-7, HeightMSL, HorizontalAccuracy, VerticalAccuracy}};
parse(?NAV, ?SOL, <<_ITOW:?U4, _FTOW:?I4, _Week:?I2, GPSFix:?U1, _/binary>>) ->
    %io:format("SOL ~p~n", [GPSFix]),
    {nav_sol, GPSFix};
%% UBX-MON-VER
parse(?MON, ?VER, <<SWVersion:30/binary, HWVersion:10/binary, Tail/binary>>) ->
    SW = hd(binary:split(SWVersion, <<0>>)),
    HW = hd(binary:split(HWVersion, <<0>>)),
    Ext = parse_extensions(Tail, []),
    {mon_ver, {SW, HW, Ext}};
%% UBX-CFG-MSG
parse(?CFG, ?MSG, <<MsgClass:?U1, MsgID:?U1, Rates/binary>>) ->
    io:format("Rate for ~p ~p: ~p~n", [MsgClass, MsgID, Rates]),
    {cfg_msg, lol};
%% UBX-TIM-TM2
parse(?TIM, ?TM2, <<Channel:?U1, NewRisingEdge:1/bits, Time:1/bits, UTC:1/bits, TimeBase:2/bits, NewFallingEdge:1/bits, Run:1/bits, Mode:1/bits,
                    Count:?U2, WNR:?U2, WNF:?U2, ToWMsR:?U4, ToWSubMsR:?U4, ToWMsF:?U4, ToWSubMsF:?U4, AccEst:?U4>>) ->
    io:format("time mark: Channel ~p, NewRisingEdge ~p, Time ~p, UTC ~p, Timebase ~p, NewFallingEdge ~p, Run ~p, Mode ~p~n", [Channel, NewRisingEdge, Time, UTC, TimeBase, NewFallingEdge, Run, Mode]),
    io:format("           Count ~p, WNR ~p WNF ~p towMsR ~p towSubMsR ~p towMsF ~p towSubMsf ~p accEst ~p~n", [Count, WNR, WNF, ToWMsR, ToWSubMsR, ToWMsF, ToWSubMsF, AccEst]),
    {tim_tm2, lol};
parse(?MON, ?HW, <<PinSel:?X4, PinBank:?X4, PinDir:?X4, PinVal:?X4, NoisePerMS:?U2, AGCCnt:?U2, AStatus:?U1, APower:?U1, _Flags:?X1, _:?U1, UsedMask:?X4, VP:17/binary, _/binary>>) ->
    io:format("Hardware Status: PinSel ~p, PinBank ~p, PinDir ~p, PinVal ~p, NoisePerMS ~p ACGCount ~p AntennaStatus ~p AntennaPower ~p~n", [PinSel, PinBank, PinDir, PinVal, NoisePerMS, AGCCnt, AStatus, APower]),
    io:format("                 used mask ~p, Pin mapping ~w~n", [UsedMask, VP]),
    {mon_hw, lol};
parse(?NAV, ?SAT, <<_ITOW:?U4, _Version:?U1, NumSatellites:?U1, _Reserved:2/binary, Tail/binary>>) ->
    io:format("Satellites in view ~p~n", [NumSatellites]),
    parse_satellites(Tail),
    {nav_sat, lol};
parse(?CFG, ?PRT, <<4:?U1, _Reserved1:?U1, TXReady:?X2, Mode:?X4, _Reserved2:4/binary, InProtoMask:?X2, OutProtoMask:?X2, Flags:?X2, _Reserved3:2/binary>>) ->
    io:format("SPI port configuration TXReady ~p Mode ~p  InProtoMask ~p OutProtoMask ~p Flags ~p~n", [TXReady, Mode, InProtoMask, OutProtoMask, Flags]),
    <<Count:9/integer-unsigned-big, PIO:5/integer-unsigned-big, POL:1/integer, EN:1/integer>> = <<TXReady:16/integer-big>>,
    io:format("TX Ready enabled: ~p Polarity ~p PIO ~p Count ~p~n", [EN, POL, PIO, Count]),
    {cfg_port, lol};
parse(?CFG, ?TP5, <<TPIdx:?U1, Version:?U1, _Reserved:2/binary, _AntennaCableDelay:?I2, _RFGroupDelay:?I2, FreqPeriod:?U4, _FreqPeriodLock:?U4, PulseLenRatio:?U4, _PulseLenRatioLock:?U4, _UserConfigDelay:?I4, TPFlags:?X4>>) ->
    io:format("Time pulse ~p, version ~p frequency ~p pulse length ~p~n", [TPIdx, Version, FreqPeriod, PulseLenRatio]),

    <<0:18/integer, SyncMode:3/integer-unsigned-big, GridUTCGNSS:4/integer-unsigned-big, ClockPolarity:1/integer, AlignToToW:1/integer, IsLength:1/integer, IsFreq:1/integer, LockedOtherSet:1/integer, LockGNSSFreq:1/integer, TPActive:1/integer>> = <<TPFlags:32/integer-unsigned-big>>,
    io:format("SyncMode ~p, Grid ~p Polarity ~p AlignToW ~p IsLength ~p IsFreq ~p, LockedOtherSet ~p LockGNSS, ~p TPActive ~p~n", [SyncMode, GridUTCGNSS, ClockPolarity, AlignToToW, IsLength, IsFreq, LockedOtherSet, LockGNSSFreq, TPActive]),
    {cfg_tp5, lol};
parse(A, B, C) ->
    io:format("unknown message 0x~.16b 0x~.16b ~p~n", [A, B, C]),
    {unknown, {A, B, C}}.


parse_extensions(<<>>, Acc) -> lists:reverse(Acc);
parse_extensions(<<Ext:30/binary, Tail/binary>>, Acc) ->
    Extension = hd(binary:split(Ext, <<0>>)),
    parse_extensions(Tail, [Extension|Acc]).
%parse_extensions(Other) ->
%io:format("Other extensions data ~p~n", [Other]).

parse_satellites(<<>>) -> ok;
parse_satellites(<<GNSSId:?U1, SvId:?U1, CNO:?U1, Elevation:?I1, Azimuth:?I2, _PrRes:?I2, _Flags:?X4, Tail/binary>>) ->
    io:format("    Satellite ID ~p ~p with C/No ~p, Elevation ~p, Azimuth ~p~n", [GNSSId, SvId, CNO, Elevation, Azimuth]),
    parse_satellites(Tail).

checksum(Binary) ->
    checksum(Binary, 0, 0).


checksum(<<>>, A, B) ->
    {A, B};

checksum(<<H:8/integer-unsigned, Tail/binary>>, A, B) ->
    A2 = ((A + H) band 16#ff),
    B2 = ((B + A2) band 16#ff),
    checksum(Tail, A2, B2).

resolve(nav_pvt) -> {?NAV, ?PVT};
resolve(nav_sol) -> {?NAV, ?SOL};
resolve(nav_sat) -> {?NAV, ?SAT};
resolve(nav_posllh) -> {?NAV, ?POSLLH};
resolve(mon_ver) -> {?MON, ?VER};
resolve(mon_hw) -> {?MON, ?HW};
resolve(cfg_port) -> {?CFG, ?PRT};
resolve(cfg_tp5) -> {?CFG, ?TP5};
resolve(tim_tm2) -> {?TIM, ?TM2}.

resolve(?NAV, ?PVT) -> nav_pvt;
resolve(?NAV, ?SOL) -> nav_sol;
resolve(?NAV, ?SAT) -> nav_sat;
resolve(?NAV, ?POSLLH) -> nav_posllh;
resolve(?MON, ?VER) -> mon_ver;
resolve(?MON, ?HW) -> mon_hw;
resolve(?CFG, ?PRT) -> cfg_port;
resolve(?CFG, ?TP5) -> cfg_tp5;
resolve(?TIM, ?TM2) -> tim_tm2.

register_ack(From, State) ->
    Ref = erlang:send_after(5000, self(), ack_timeout),
    State#state{ack={From, Ref}}.

register_poll(Msg, From, State) ->
    Ref = erlang:send_after(5000, self(), poll_timeout),
    State#state{poll={Msg, From, Ref}}.
