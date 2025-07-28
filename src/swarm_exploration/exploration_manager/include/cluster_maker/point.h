#ifndef _POINT_H_
#define _POINT_H_

#include <iostream>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <algorithm>

class Point {
private:
  int id_point, id_cluster;
  vector<double> values;
  int total_values;
  string name;

public:
  Point(int id_point, vector<double>& values, string name = "") {
    this->id_point = id_point;
    total_values = values.size();

    for (int i = 0; i < total_values; i++) this->values.push_back(values[i]);

    this->name = name;
    id_cluster = -1;
  }

  int getID() {
    return id_point;
  }

  void setCluster(int id_cluster) {
    this->id_cluster = id_cluster;
  }

  int getCluster() {
    return id_cluster;
  }

  double getValue(int index) {
    return values[index];
  }

  int getTotalValues() {
    return total_values;
  }

  void addValue(double value) {
    values.push_back(value);
  }

  string getName() {
    return name;
  }
};

#endif
