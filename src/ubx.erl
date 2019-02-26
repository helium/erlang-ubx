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
-define(MGA, 16#13).

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
-define(TIMEUTC, 16#21).
-define(INI_TIME_UTC, 16#40).
-define(ANO, 16#20).
-define(NAV5, 16#24).
-define(NAVX5, 16#23).
-define(ACK_DATA0, 16#60).


-type fix_type() :: non_neg_integer(). %% gps fix type
-type nav_sol() :: fix_type().
-type nav_pvt() :: #{
                     fix_type => fix_type(),
                     num_sats => pos_integer(), %% number of visible sats
                     lat => float(), %% latitude in degrees
                     lon => float(), %% longitude in degrees
                     height_msl => integer(), %% height mean sea level in MM
                     height => integer(), %% height in MM
                     h_acc => non_neg_integer(), %% in MM
                     v_acc => non_neg_integer(), %% in MM
                     t_acc => non_neg_integer()
                    }.
-type nav_sat() :: #{
                     id => non_neg_integer(), %% GNSS Id
                     sv_id => non_neg_integer(),
                     cno => non_neg_integer(),
                     elevation => integer(),
                     azimuth => integer()
                    }.
-type nav_posllh() :: #{
                        lat => float(), %% latitude in degrees
                        lon => float(), %% longitude in degrees
                        h_acc => non_neg_integer(), %% in MM
                        v_acc => non_neg_integer() %% in MM
                       }.
-type nav_timeutc() :: #{
                         datetime => calendar:datetime(),
                         t_acc => non_neg_integer(), %% time accuracy estimate in ns
                         nano => integer(), %% fraction of sec, range -1e9 .. 1e9,
                         utc_std => non_neg_integer(), %% UTC standard identifier
                         valid_utc => non_neg_integer(), %% 1 = Valid UTC Time
                         valid_wkn => non_neg_integer(), %% 1 = Valid Week Number
                         valid_tow => non_neg_integer()  %% 1 = Valid Time of Week
                        }.

-export_type([nav_pvt/0, nav_sat/0, nav_posllh/0, nav_sol/0, nav_timeutc/0, fix_type/0]).

-record(state, {
          device :: port() | pid(),
          buffer = <<>> :: binary(),
          controlling_process :: pid(),
          ack,
          poll,
          gpio,
          gpionum
         }).

-export([start_link/4, enable_message/3, disable_message/2, fix_type/1, stop/2,
         poll_message/2, poll_message/3, set_time_utc/2, parse/1,
         upload_offline_assistance/2, upload_online_assistance/2]).

-export([init/1, handle_info/2, handle_cast/2, handle_call/3]).

start_link(Filename, GpioNum, Options, ControllingProcess) ->
    gen_server:start_link(?MODULE, [Filename, GpioNum, Options, ControllingProcess], []).

enable_message(Pid, Msg, Frequency) ->
    {MsgClass, MsgID} = resolve(Msg),
    gen_server:call(Pid, {enable_message, MsgClass, MsgID, Frequency}).

disable_message(Pid, Msg) ->
    {MsgClass, MsgID} = resolve(Msg),
    gen_server:call(Pid, {enable_message, MsgClass, MsgID, 0}).

poll_message(Pid, Msg) ->
    {MsgClass, MsgID} = resolve(Msg),
    gen_server:call(Pid, {poll_message, MsgClass, MsgID, <<>>}).

poll_message(Pid, Msg, Payload) ->
    {MsgClass, MsgID} = resolve(Msg),
    gen_server:call(Pid, {poll_message, MsgClass, MsgID, Payload}).

set_time_utc(Pid, DateTime) ->
    gen_server:call(Pid, {set_time_utc, DateTime}).

upload_online_assistance(Pid, File) ->
    gen_server:call(Pid, {upload_online_assistance, File}, infinity).

stop(Pid, Reason) ->
    gen_server:stop(Pid, Reason, infinity).

upload_offline_assistance(Pid, File) ->
    gen_server:call(Pid, {upload_offline_assistance, File}, infinity).

init([Filename, GpioNum, Options, ControllingProcess]) ->
    {ok, Spi} = spi:start_link(Filename, Options),
    {ok, Gpio} = gpio:start_link(GpioNum, input),
    State0 = #state{device=Spi, controlling_process=ControllingProcess, gpio=Gpio, gpionum=GpioNum},
    gpio:register_int(Gpio),
    gpio:set_int(Gpio, rising),
    %% flush any buffered data
    flush_tx_buf(State0),
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
    PulseLenRatioLock = 0, % disabled for now 2147483648, %% 50% duty cycle when locked
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
    %% Enable flow control
    send(NewState2, frame(?CFG, ?NAVX5, <<2:?U2, (1 bsl 10):?X2, 0:?X4, 0:(9*8)/integer, 1:?U1, 0:(22*8)/integer>>)),
    {NewState3, {ack, ?CFG, ?NAVX5}} = get_ack(NewState2),
    %% PIO changes don't take effect until a config save
    send(NewState3, frame(?CFG, ?CFG2, <<0:?X4, 16#ff, 16#ff, 0, 0, 0:?X4, 23:?X1>>)),
    {NewState4, {ack, ?CFG, ?CFG2}} = get_ack(NewState3),
    case gpio:read(Gpio) of
        1 ->
            self() ! {gpio_interrupt,GpioNum,rising};
        0 ->
            ok
    end,
    {ok, NewState4}.

handle_info({gpio_interrupt,GpioNum,rising}, State = #state{ack=Ack, poll=Poll, gpionum=GpioNum}) ->
    %io:format("handling interrupt~n"),
    {ok, NewerState} = case get_packet(State) of
        {NewState, {error, Error}} ->
                               io:format("error ~p~n", [Error]),
                        {ok, NewState};
        {NewState, {ack, ?CFG, _}} ->
                        io:format("ack~n"),
            case Ack of
                {From, Ref} ->
                    erlang:cancel_timer(Ref),
                    gen_server:reply(From, ok),
                    {ok, NewState#state{ack=undefined}};
                undefined ->
                    {ok, NewState}
            end;
        {NewState, {nack, ?CFG, _}} ->
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
            handle_info({gpio_interrupt, GpioNum, rising}, NewerState);
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
            {reply, {error, busy}, State}
    end;

handle_call({set_time_utc, DateTime}, _From, State) ->
    case DateTime of
        {{Year, Month, Day}, {Hour, Minute, Second}} ->
            Type = 16#10,
            Version = 16#00,
            Source = 0, %% 0: none, i.e. on receipt of message (will be inaccurate!)
            Fall = 0, %% use falling edge of EXTINT pulse (default rising) - only if source is EXTINT
            Last = 0, %% use last EXTINT pulse (default next pulse) - only if source is EXTINT
            Ref = << 0:1/integer, 0:1/integer, Last:1/integer, Fall:1/integer, Source:4/integer-unsigned-big>>,
            LeapSecs = -128, %% number of leap seconds since 1980 (-128 if unknown)
            Reserved1 = <<0>>,
            Nanosecs = 0,
            TAccS = 0, %% seconds part of time accuracy
            Reserved2 = <<0, 0>>,
            TAccNs = 500000000, %% nanoseconds part of time accuracy
            Packet = <<Type:?U1, Version:?U1, Ref/binary, LeapSecs:?I1, Year:?U2, Month:?U1, Day:?U1, Hour:?U1, Minute:?U1, Second:?U1, Reserved1/binary, Nanosecs:?U4, TAccS:?U2, Reserved2/binary, TAccNs:?U4>>,
            send(State, frame(?MGA, ?INI_TIME_UTC, Packet)),
            {reply, ok, State};
        _ ->
            {reply, {error, invalid_datetime}, State}
    end;
handle_call({upload_offline_assistance, Filename}, _From, State) ->
    {ok, Bin} = file:read_file(Filename),
    {{Year, Month, Day}, _} = calendar:universal_time(),
    Msgs = find_matching_assistance_messages(Bin, Year rem 100, Month, Day, []),
    %io:format("Matching messages ~p~n", [Res]),
    [ensure_send(State, Msg) || Msg <- Msgs],
    {reply, ok, State};
handle_call({upload_online_assistance, Filename}, _From, State) ->
    {ok, Bin} = file:read_file(Filename),
    Msgs = bin_to_messages(Bin, []),
    [ensure_send(State, Msg) || Msg <- Msgs],
    {reply, ok, State};
handle_call(Msg, _From, State) ->
    {reply, {unknown_call, Msg}, State}.

send(_, <<>>) ->
    ok;
send(S=#state{device=Device}, <<Packet:128/binary, Tail/binary>>) ->
    io:format("Sending~s~n", [lists:flatten([ io_lib:format(" ~.16b", [X]) || <<X:8/integer>> <= Packet ])]),
    spi:transfer(Device, Packet),
    send(S, Tail);
send(#state{device=Device}, Packet) ->
    io:format("Sending~s~n", [lists:flatten([ io_lib:format(" ~.16b", [X]) || <<X:8/integer>> <= Packet ])]),
    spi:transfer(Device, Packet).

ensure_send(State, <<>>) ->
    {reply, ok, State};
ensure_send(State, Msg) ->
    send(State, Msg),
    case get_packet(State, <<>>, 10000) of
        {NewState, {mga_ack, _} = MgaAck} ->
            case MgaAck of
                {mga_ack, {1, 0, ID, PayloadStart}} ->
                    io:format("assistance accepted ID ~p PayloadStart ~p~n", [ID, PayloadStart]);
                {mga_ack, {Type, InfoCode, ID, PayloadStart}} ->
                    io:format("assistance rejected ID ~p PayloadStart ~p Type ~p InfoCode ~p~n", [ID, PayloadStart, Type, InfoCode])
            end,
            {NewState, MgaAck};
        {NewState, {error, Error}} ->
            io:format("Error ~p; resending assistance data~n", [Error]),
            ensure_send(NewState, Msg);
        {NewState, Other} ->
            io:format("non-mga_ack packet ~p; resending assistance data~n", [Other]),
            ensure_send(NewState, Msg)
    end.

recv(State=#state{device=Device}, Length) when Length =< 128 ->
    %io:format("SPI read ~p~n", [Length]),
    {State, spi:transfer(Device, binary:copy(<<255>>, Length))};
recv(State, Length) ->
    %% length over 128
    recv_loop(State, Length, []).

recv_loop(State, Length, Acc) when Length =< 128 ->
    {NewState, Bin} = recv(State, Length),
    {NewState, list_to_binary(lists:reverse([Bin|Acc]))};
recv_loop(State, Length, Acc) ->
    {NewState, Bin} = recv(State, 128),
    recv_loop(NewState, Length - 128, [Bin|Acc]).

flush_tx_buf(State) ->
    {NewState, Bin} = recv(State, 128),
    %% wait until only 255s come back
    case binary:copy(<<255>>, 128) == Bin of
        true ->
            NewState;
        false ->
            flush_tx_buf(NewState)
    end.

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
            io:format("non-ack or non-nack packet ~p~n", [Other]),
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
                    {NewState2, Body} = recv(NewState, Length + 2),
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
    end;
parse(<<?HEADER1, ?HEADER2, Class:?U1, ID:?U1, Length:?U2, Tail/binary>>) ->
    io:format("Got packet ~p  with declared length ~p but actual length ~p~n", [catch(resolve(Class, ID)), Length, byte_size(Tail) - 2]),
    {error, bad_length}.

%% UBX-ACK-ACK
parse(16#5, 16#1, <<ClassID:?U1, MsgID:?U1>>) ->
    {ack, ClassID, MsgID};

%% UBX-ACK-NACK
parse(16#5, 16#0, <<ClassID:?U1, MsgID:?U1>>) ->
    {nack, ClassID, MsgID};

%% UBX-NAV-PVT
parse(?NAV, ?PVT, <<_ITOW:?U4, _Year:?U2, _Month:?U1, _Day:?U1, _Hour:?U1, _Min:?U1, _Sec:?U1,
                    _Valid:?X1, TimeAccuracy:?U4, _Nano:?I4, FixType:?U1, _Flags:?X1, _Flags2:?X1,
                    NumSV:?U1, Longitude:?I4, Latitude:?I4, Height:?I4, HeightMSL:?I4, HorizontalAccuracy:?U4,
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
    {nav_pvt, #{
                fix_type => FixType,
                num_sats => NumSV,
                lat => Latitude * 1.0e-7,
                lon => Longitude * 1.0e-7,
                height_msl => HeightMSL,
                height => Height,
                h_acc => HorizontalAccuracy,
                v_acc => VerticalAccuracy,
                t_acc => TimeAccuracy
               }};
%% UBX-NAV-POSLLH
parse(?NAV, ?POSLLH, <<_ITOW:?U4, Longitude:?I4, Latitude:?I4, _Height:?I4, _HeightMSL:?I4, HorizontalAccuracy:?U4, VerticalAccuracy:?U4>>) ->
    %io:format("Longitude ~f, Latitude ~f, Height Ellipsoid ~p ft, Height MeanSeaLevel ~f ft, Horizontal Accuracy ~f ft, Vertical Accuracy ~f~n",
    %[Longitude * 1.0e-7, Latitude * 1.0e-7, Height * ?MM_TO_FEET, HeightMSL * ?MM_TO_FEET, HorizontalAccuracy * ?MM_TO_FEET, VerticalAccuracy * ?MM_TO_FEET]),
    {nav_posllh, #{
                   lat => Latitude * 1.0e-7,
                   lon => Longitude * 1.0e-7,
                   h_acc => HorizontalAccuracy,
                   v_acc => VerticalAccuracy
                  }};
parse(?NAV, ?SOL, <<_ITOW:?U4, _FTOW:?I4, _Week:?I2, GPSFix:?U1, _/binary>>) ->
    %io:format("SOL ~p~n", [GPSFix]),
    {nav_sol, GPSFix};
%% UBX-NAV-TIMEUTC
parse(?NAV, ?TIMEUTC, <<_ITOW:?U4, TAcc:?U4, Nano:?I4, Year:?U2, Month:?U1, Day:?U1, Hour:?U1, Min:?U1, Sec:?U1, Valid:?X1>>) ->
    io:format("Time UTC: Year ~p Month ~p Day ~p Hour ~p Min ~p Sec ~p TAcc ~p Nano ~p~n", [Year, Month, Day, Hour, Min, Sec, TAcc, Nano]),
    <<UTCStandard:4/integer-unsigned-big, _:1/integer-unsigned, ValidUTC:1/integer-unsigned, ValidWKN:1/integer-unsigned, ValidTOW:1/integer-unsigned>> = <<Valid:8/integer-big>>,
    io:format("          UTCStandard ~p ValidUTC ~p ValidWKN ~p ValidTOW ~p~n", [UTCStandard, ValidUTC, ValidWKN, ValidTOW]),
    {nav_timeutc, #{
                    datetime => {{Year, Month, Day}, {Hour, Min, Sec}},
                    t_acc => TAcc,
                    nano => Nano,
                    utc_std => UTCStandard,
                    valid_utc => ValidUTC,
                    valid_wkn => ValidWKN,
                    valid_tow => ValidTOW
                   }};
%% UBX-MON-VER
parse(?MON, ?VER, <<SWVersion:30/binary, HWVersion:10/binary, Tail/binary>>) ->
    SW = hd(binary:split(SWVersion, <<0>>)),
    HW = hd(binary:split(HWVersion, <<0>>)),
    Ext = parse_extensions(Tail, []),
    {mon_ver, {SW, HW, Ext}};
%% UBX-MGA-ACK-DATA0
parse(?MGA, ?ACK_DATA0, <<Type:?U1, 0, InfoCode:?U1, ID:?U1, PayloadStart:4/binary>>) ->
    io:format("Type ~p InfoCode ~p ID ~p PayloadStart ~p~n", [Type, InfoCode, ID, PayloadStart]),
    {mga_ack, {Type, InfoCode, ID, PayloadStart}};
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
parse(?NAV, ?SAT, <<_ITOW:?U4, _Version:?U1, _NumSatellites:?U1, _Reserved:2/binary, Tail/binary>>) ->
    {nav_sat, parse_satellites(Tail, [])};
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
parse(?CFG, ?NAV5, <<Mask:?X2, DynModel:?U1, FixMode:?U1, _Tail/binary>>) ->
    io:format("Mask ~w~n", [Mask]),
    io:format("Dynamic platform model ~p, Fix mode ~p~n", [DynModel, FixMode]),
    {cfg_nav5, lol};
parse(?CFG, ?NAVX5, <<Version:?U2, _Mask1:?X2, _Mask2:?X4, _Reserved1:2/binary, _MinSVs:?U1, _MaxSVs:?U1, _MinCNO:?U1, _Reserved2:?U1, _InFix3D:?U1, _Reserved3:2/binary, AckAiding:?U1, _WknRollover:?U2, _SigAttenCompMode:?U1, _Reserved4:5/binary, _UsePPP:?U1, _AopCfg:?U1, _Reserved7:2/binary, _AopOrbMaxErr:?U2, _Reserved8:7/binary, _UseAdr:?U1>>) ->
    io:format("Version ~p AckAiding ~p~n", [Version, AckAiding]),
    {cfg_navx5, lol};
parse(A, B, C) ->
    io:format("unknown message 0x~.16b 0x~.16b ~p~n", [A, B, C]),
    {unknown, {A, B, C}}.


parse_extensions(<<>>, Acc) -> lists:reverse(Acc);
parse_extensions(<<Ext:30/binary, Tail/binary>>, Acc) ->
    Extension = hd(binary:split(Ext, <<0>>)),
    parse_extensions(Tail, [Extension|Acc]).
%parse_extensions(Other) ->
%io:format("Other extensions data ~p~n", [Other]).

parse_satellites(<<>>, Acc) ->
    lists:reverse(Acc);
parse_satellites(<<GNSSId:?U1, SvId:?U1, CNO:?U1, Elevation:?I1, Azimuth:?I2, _PrRes:?I2, Flags:?X4, Tail/binary>>, Acc) ->

    <<0:9/integer, _DoCorrUsed:1/integer, _CrCorrUsed:1/integer, _PrCorrUsed:1/integer,
      _:1/integer, _SLASCorrUsed:1/integer, _RTCMCorrUsed:1/integer, _SBASCorrUsed:1/integer,
      _:1/integer, _AOPAvail:1/integer, _ANOAvail:1/integer, _AlmanacAvail:1/integer,
      _EphemerisAvail:1/integer, OrbitSource:1/integer, _Reserved:2/integer-unsigned-big, _Smoothed:1/integer,
      _DiffCorr:1/integer, Health:2/integer-unsigned-big, SVUsed:1/integer,
      Quality:3/integer-unsigned-big>> = <<Flags:32/integer-unsigned-big>>,
    Info = #{
             type => sat_id(GNSSId),
             id => SvId,
             cno => CNO,
             health => Health,
             quality => Quality,
             used => SVUsed == 1,
             elevation => Elevation,
             azimuth => Azimuth,
             orbit => OrbitSource},
    parse_satellites(Tail, [Info | Acc]).

sat_id(0) ->
    gps;
sat_id(1) ->
    sbas;
sat_id(2) ->
    galileo;
sat_id(3) ->
    beidou;
sat_id(4) ->
    imes;
sat_id(5) ->
    qzss;
sat_id(6) ->
    glonass.


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
resolve(nav_timeutc) -> {?NAV, ?TIMEUTC};
resolve(mon_ver) -> {?MON, ?VER};
resolve(mon_hw) -> {?MON, ?HW};
resolve(cfg_port) -> {?CFG, ?PRT};
resolve(cfg_tp5) -> {?CFG, ?TP5};
resolve(cfg_nav5) -> {?CFG, ?NAV5};
resolve(cfg_navx5) -> {?CFG, ?NAVX5};
resolve(cfg_cfg) -> {?CFG, ?CFG2};
resolve(tim_tm2) -> {?TIM, ?TM2};
resolve(mga_ack) -> {?MGA, ?ACK_DATA0}.

resolve(?NAV, ?PVT) -> nav_pvt;
resolve(?NAV, ?SOL) -> nav_sol;
resolve(?NAV, ?SAT) -> nav_sat;
resolve(?NAV, ?POSLLH) -> nav_posllh;
resolve(?NAV, ?TIMEUTC) -> nav_timeutc;
resolve(?MON, ?VER) -> mon_ver;
resolve(?MON, ?HW) -> mon_hw;
resolve(?CFG, ?PRT) -> cfg_port;
resolve(?CFG, ?TP5) -> cfg_tp5;
resolve(?CFG, ?NAV5) -> cfg_nav5;
resolve(?CFG, ?NAVX5) -> cfg_navx5;
resolve(?CFG, ?CFG2) -> cfg_cfg;
resolve(?TIM, ?TM2) -> tim_tm2;
resolve(?MGA, ?ACK_DATA0) -> mga_ack.

register_ack(From, State) ->
    Ref = erlang:send_after(5000, self(), ack_timeout),
    State#state{ack={From, Ref}}.

register_poll(Msg, From, State) ->
    Ref = erlang:send_after(5000, self(), poll_timeout),
    State#state{poll={Msg, From, Ref}}.

fix_type(Byte) ->
    case Byte of
        0 -> no_fix;
        1 -> dead_reckoning;
        2 -> fix_2d;
        3 -> fix_3d;
        4 -> gnss_and_dead_reckoning;
        5 -> time_only
    end.

find_matching_assistance_messages(<<>>, _, _, _, Acc) ->
    lists:reverse(Acc);
find_matching_assistance_messages(<<?HEADER1, ?HEADER2, ?MGA:?U1, ?ANO:?U1, Length:?U2, Body:Length/binary, CK_A:?U1, CK_B:?U1, Tail/binary>>, Year, Month, Day, Acc) ->
    case checksum(<<?MGA:?U1, ?ANO:?U1, Length:?U2, Body/binary>>) of
        {CK_A, CK_B} ->
            %% checksum is OK
            case Body of
                <<0:?U1, 0:?U1, _SVId:?U1, _GNSSId:?U1, Year:?U1, Month:?U1, Day:?U1, _/binary>> ->
                    %% got a match
                    find_matching_assistance_messages(Tail, Year, Month, Day,
                                                      [<<?HEADER1, ?HEADER2, ?MGA:?U1, ?ANO:?U1, Length:?U2, Body:Length/binary, CK_A:?U1, CK_B:?U1>>|Acc]);
                _ ->
                    find_matching_assistance_messages(Tail, Year, Month, Day, Acc)
            end;
        _Other ->
            find_matching_assistance_messages(Tail, Year, Month, Day, Acc)
    end.

bin_to_messages(<<>>, Acc) ->
    lists:reverse(Acc);
bin_to_messages(<<?HEADER1, ?HEADER2, ?MGA:?U1, ID:?U1, Length:?U2, Body:Length/binary, CK_A:?U1, CK_B:?U1, Tail/binary>>, Acc) ->
    case checksum(<<?MGA:?U1, ID:?U1, Length:?U2, Body/binary>>) of
        {CK_A, CK_B} ->
            %% checksum is OK
            bin_to_messages(Tail,
                [<<?HEADER1, ?HEADER2, ?MGA:?U1, ID:?U1, Length:?U2, Body:Length/binary, CK_A:?U1, CK_B:?U1>>|Acc]);
        _ ->
            io:format("dropping message length: ~p", [Length]),
            bin_to_messages(Tail, Acc)
    end.
