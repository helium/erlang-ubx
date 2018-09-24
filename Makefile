.PHONY: deps compile

REBAR=./rebar3

deps:
	$(REBAR) get-deps
	$(REBAR) upgrade
	$(REBAR) lock

compile:
	$(REBAR) compile

clean:
	$(REBAR) clean

