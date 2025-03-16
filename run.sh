#!/bin/bash
export LD_LIBRARY_PATH=./lib:$LD_LIBRARY_PATH
javac -d . java/FaceProcessor.java
java -Djava.library.path=./lib FaceProcessor
