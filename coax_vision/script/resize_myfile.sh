#!/bin/bash
for f in *.jpg;
	do echo $f && convert -size 2880x864 "$f" -resize 1440x432 "$f";
done
