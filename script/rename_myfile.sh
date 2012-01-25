#!/bin/bash
for f in [0-9]*; 
	do mv $f `printf %04d.jpg ${f%.jpg}` ;
done
