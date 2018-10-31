.PHONY: compile release

REBAR=./rebar3

compile:
	$(REBAR) compile

clean:
	$(REBAR) clean

release:
	$(REBAR) release -d false

