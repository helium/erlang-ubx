.PHONY: compile release

REBAR=./rebar3

compile:
	$(REBAR) compile

clean:
	$(REBAR) clean

rebar:
	$(REBAR) release -d false

