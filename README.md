# GeoPathFinder

## Description

This is a project about finding most cost-efficient path for surveilleance drone over a geographial area. Also in certain scenarios, it can be benegicial to determine the most optimal route for traveling on land. For example when traversing mountainous terrain on foot, you may want to identify the path that result in THE LEAST OVERALL CHANGE in elevation.

I make my project to show both solutions on map as a draw. Green line means most cost-efficient route and yellow line means least-elevation change route.

I acquired from this project is applying Dijkstra's Algorithm to solve shortest path problems.

## Installation, Run and requirements

- OpenJDK 11

Clone the project and than run these commands.

- For Windows Powershell:
  - javac -cp _.jar _.java -d .
  - java -cp '.;\*' Main sample_input.dat
- For Windows CMD:
  - javac -cp _.jar _.java -d .
  - java -cp .;\* Main sample_input.dat
- For Windows linux:
  - javac -cp *.jar *.java -d .
  - java -cp .:* Main input_arguments.dat
