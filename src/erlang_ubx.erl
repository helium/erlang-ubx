-module(erlang_ubx).
-behaviour(gen_server).

-define(U1, 8/integer-unsigned). %% unsigned char
-define(I1, 8/integer-signed). %% signed char
-define(X1, 8/integer). %% bitfield
-define(U2, 16/integer-unsigned-little). %% unsigned short
-define(I2, 16/integer-signed-little). %% signed short
-define(X2, 16/integer-little). %% bitfields are little endian
-define(U4, 32/integer-unsigned-little). %% unsigned long
-define(I4, 32/integer-signed-little). %% signed long
-define(X4, 32/integer-little). %% bitfields are little endian
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

-record(state, {
          device :: port() | pid(),
          devicetype :: spi | serial,
          buffer = <<>> :: binary(),
          controlling_process :: pid()
         }).

-export([start_link/4, enable_message/3, disable_message/2, poll_message/2]).

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
    gen_server:call(Pid, {poll_message, MsgClass, MsgID}).

init([spi, Filename, Options, ControllingProcess]) ->
    {ok, Spi} = spi:start_link(Filename, Options),
    State = #state{device=Spi, devicetype=spi, controlling_process=ControllingProcess},
    %% only UBX on spi port
    send(State, <<16#b5, 16#62, 16#6, 0, 16#14, 0, 4, 0, 0, 0, 0, 16#32, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 16#52, 16#94>>),
    {NewState, {ack, ?CFG, ?PRT}} = get_ack(State),
    {ok, NewState};
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

handle_info({data, Bytes}, State) ->
    case get_packet(State#state{buffer= <<(State#state.buffer)/binary, Bytes/binary>>}) of
        {NewState, {error, _}} ->
            {noreply, NewState};
        {NewState, Packet} ->
            State#state.controlling_process ! Packet,
            {noreply, NewState}
    end.


handle_cast(_Msg, State) ->
    {noreply, State}.

handle_call({enable_message, MsgClass, MsgID, Frequency}, _From, State) ->
    send(State, frame(?CFG, ?MSG, <<MsgClass, MsgID, Frequency>>)),
    case get_ack(State) of
        {NewState, {ack, ?CFG, ?MSG}} ->
            {reply, ok, NewState};
        {NewState, {nack, ?CFG, ?MSG}} ->
            {reply, {error, nack}, NewState};
        {NewState, {error, Reason}} ->
            {reply, {error, Reason}, NewState}
    end;
handle_call({poll_message, MsgClass, MsgID}, _From, State) ->
    Msg = resolve(MsgClass, MsgID),
    send(State, frame(MsgClass, MsgID, <<>>)),
    case get_packet(State) of
        {NewState, {Msg, Data}} ->
            {reply, {Msg, Data}, NewState};
        {NewState, {error, Reason}} ->
            {reply, {error, Reason}, NewState};
        {NewState, {OtherMsg, Data}} ->
            NewState#state.controlling_process ! {OtherMsg, Data},
            {reply, {error, timeout}, NewState}
    end;

handle_call(Msg, _From, State) ->
    {reply, {unknown_call, Msg}, State}.

send(#state{devicetype=spi, device=Device}, Packet) ->
    spi:transfer(Device, Packet);
send(#state{devicetype=serial, device=Device}, Packet) ->
    Device ! {send, Packet}.

recv(State=#state{devicetype=spi, device=Device}, Length) ->
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
    case get_packet(State) of
        {NewState, {ack, _, _} = Ack} ->
            {NewState, Ack};
        {NewState, {nack, _, _} = Nack} ->
            {NewState, Nack};
        {NewState, {error, _} = Error} ->
            {NewState, Error};
        {NewState, Other} ->
            NewState#state.controlling_process ! {packet, Other},
            get_ack(NewState)
    end.

get_packet(State) ->
    get_packet(State, <<>>, 10).

get_packet(State, _, 0) ->
    {State, {error, timeout}};
get_packet(State, Acc, Count) ->
    %io:format("reading ~p bytes~n", [6 - byte_size(Acc)]),
    {NewState, Reply} = recv(State, 6 - byte_size(Acc)),
    case Reply of
        {error, _} ->
            {NewState, Reply};
        _ ->
            case list_to_binary([Reply, Acc]) of
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
                <<?HEADER1, ?HEADER2, Class:?U1, ID:?U1, Length:?U2>> = Header ->
                    {NewState2, Body} = case is_spi(State) andalso Length + 2 > 255 of
                                            true ->
                                                {_, B1} = recv(NewState, 255),
                                                {_, B2} = recv(NewState, (Length + 2) rem 255),
                                                {NewState, <<B1/binary, B2/binary>>};
                                            false ->
                                                recv(NewState, Length + 2)
                                        end,
                    {NewState2, parse(<<Header/binary, Body/binary>>)};
                Other ->
                    io:format("goop ~w~n", [Other]),
                    {NewState, {error, {goop, Other}}}
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
parse(?NAV, ?PVT, <<ITOW:?U4, Year:?U2, Month:?U1, Day:?U1, Hour:?U1, Min:?U1, Sec:?U1,
                    Valid:?X1, TimeAccuracy:?U4, Nano:?I4, FixType:?U1, Flags:?X1, Flags2:?X1,
                    NumSV:?U1, Longitude:?I4, Latitude:?I4, Height:?I4, HeightMSL:?I4, HorizontalAccuracy:?U4,
                    VerticalAccuracy:?U4, VelocityN:?I4, VelocityE:?I4, VelocityD:?I4, Speed:?I4,
                    Heading:?I4, SpeedAccuracy:?U4, HeadingAccuracy:?U4, PositionDOP:?U2, _:6/binary,
                    VehicleHeading:?I4, MagneticDeclination:?I2, MagneticAccuracy:?U2>>) ->
    io:format("ITOW ~p, Year ~p, Month ~p, Day ~p, Hour ~p, Minute ~p, Second ~p~n",
              [ITOW, Year, Month, Day, Hour, Min, Sec]),
    io:format("Valid ~p, TimeAccuracy ~p, Nano ~p, FixTime ~p, Flags ~p, Flags2 ~p~n",
              [Valid, TimeAccuracy, Nano, FixType, Flags, Flags2]),
    io:format("NumSatellites ~p, Longitude ~f, Latitude ~f, Height Ellipsoid ~p ft, Height MeanSeaLevel ~f ft, Horizontal Accuracy ~f ft~n",
              [NumSV, Longitude * 1.0e-7, Latitude * 1.0e-7, Height * ?MM_TO_FEET, HeightMSL * ?MM_TO_FEET, HorizontalAccuracy * ?MM_TO_FEET]),
    io:format("Vertical Accuracy ~f ft, Velocity North ~f mph, Velocity East ~f mph, Velocity Down ~f mph, Ground Speed ~f mph~n", 
              [VerticalAccuracy * ?MM_TO_FEET, VelocityN * ?MM_TO_MILES, VelocityE * ?MM_TO_MILES, VelocityD * ?MM_TO_MILES, Speed * ?MM_TO_MILES]),
    {nav_pvt, {Latitude * 1.0e-7, Longitude * 1.0e-7, HeightMSL, HorizontalAccuracy, VerticalAccuracy}};
%% UBX-NAV-POSLLH
parse(?NAV, ?POSLLH, <<ITOW:?U4, Longitude:?I4, Latitude:?I4, Height:?I4, HeightMSL:?I4, HorizontalAccuracy:?U4, VerticalAccuracy:?U4>>) ->
    %io:format("Longitude ~f, Latitude ~f, Height Ellipsoid ~p ft, Height MeanSeaLevel ~f ft, Horizontal Accuracy ~f ft, Vertical Accuracy ~f~n",
    %[Longitude * 1.0e-7, Latitude * 1.0e-7, Height * ?MM_TO_FEET, HeightMSL * ?MM_TO_FEET, HorizontalAccuracy * ?MM_TO_FEET, VerticalAccuracy * ?MM_TO_FEET]),
    {nav_posllh, {Latitude * 1.0e-7, Longitude * 1.0e-7, HeightMSL, HorizontalAccuracy, VerticalAccuracy}};
parse(?NAV, ?SOL, <<ITOW:?U4, FTOW:?I4, Week:?I2, GPSFix:?U1, _/binary>>) ->
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
parse(?MON, ?HW, <<PinSel:?X4, PinBank:?X4, PinDir:?X4, PinVal:?X4, NoisePerMS:?U2, AGCCnt:?U2, AStatus:?U1, APower:?U1, Flags:?X1, _:?U1, UsedMask:?X4, VP:17/binary, _/binary>>) ->
    io:format("Hardware Status: PinSel ~p, PinBank ~p, PinDir ~p, PinVal ~p, NoisePerMS ~p ACGCount ~p AntennaStatus ~p AntennaPower ~p~n", [PinSel, PinBank, PinDir, PinVal, NoisePerMS, AGCCnt, AStatus, APower]),
    io:format("                 used mask ~p, Pin mapping ~w~n", [UsedMask, VP]),
    {mon_hw, lol};
parse(?NAV, ?SAT, <<ITOW:?U4, Version:?U1, NumSatellites:?U1, _Reserved:2/binary, Tail/binary>>) ->
    io:format("Satellites in view ~p~n", [NumSatellites]),
    parse_satellites(Tail),
    {nav_sat, lol};
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
parse_satellites(<<GNSSId:?U1, SvId:?U1, CNO:?U1, Elevation:?I1, Azimuth:?I2, PrRes:?I2, Flags:?X4, Tail/binary>>) ->
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
resolve(nav_posllh) -> {?NAV, ?POSLLH};
resolve(mon_ver) -> {?MON, ?VER}.

resolve(?NAV, ?PVT) -> nav_pvt;
resolve(?NAV, ?SOL) -> nav_sol;
resolve(?NAV, ?POSLLH) -> nav_posllh;
resolve(?MON, ?VER) -> mon_ver.
