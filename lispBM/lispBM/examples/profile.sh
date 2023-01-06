#!/bin/bash


valgrind --tool=callgrind --callgrind-out-file=cg.out ${*:1}

gprof2dot -f callgrind cg.out -o cg.dot

dot -Tpdf cg.dot -o cg.pdf

rm cg.out
rm cg.dot
