#!/bin/bash

cp $1 a

tr -d "[a-z:_/ ,]" < a > b
sed 's/0.00.00.00.00.00.00.00.00.0//' b > a
tr  -s "\n" "," < a > b
sed 's/,---,/#/g' b > a
tr  "#" "\n" < a > b
sed 's/^,//g' b > a

cp a $2
rm a b
