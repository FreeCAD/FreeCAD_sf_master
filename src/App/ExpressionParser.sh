#!/usr/bin/env sh

cd "$(dirname "$0")"

flex -v -olex.ExpressionParser.c < ExpressionParser.l
bison -v -Wall -oExpressionParser.tab.c ExpressionParser.y
