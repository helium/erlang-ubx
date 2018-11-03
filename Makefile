.PHONY: compile test release

REBAR=./rebar3

compile:
	$(REBAR) compile

clean:
	$(REBAR) clean

test:
	$(REBAR) ct

release:
	$(REBAR) release -d false

