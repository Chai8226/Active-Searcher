// Implementation of the KMeans Algorithm
// reference: http://mnemstudio.org/clustering-k-means-example-1.htm
#ifndef _KMEANS_CLUSTER_H_
#define _KMEANS_CLUSTER_H_

#include <iostream>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include <point.h>

using namespace std;

class Cluster {
private:
  int id_cluster;
  vector<double> central_values;
  vector<Point> points;

public:
  Cluster(int id_cluster, Point point) {
    this->id_cluster = id_cluster;

    int total_values = point.getTotalValues();

    for (int i = 0; i < total_values; i++) central_values.push_back(point.getValue(i));

    points.push_back(point);
  }

  void addPoint(Point point) {
    points.push_back(point);
  }

  bool removePoint(int id_point) {
    int total_points = points.size();

    for (int i = 0; i < total_points; i++) {
      if (points[i].getID() == id_point) {
        points.erase(points.begin() + i);
        return true;
      }
    }
    return false;
  }

  double getCentralValue(int index) {
    return central_values[index];
  }

  void setCentralValue(int index, double value) {
    central_values[index] = value;
  }

  Point getPoint(int index) {
    return points[index];
  }

  int getTotalPoints() {
    return points.size();
  }

  int getID() {
    return id_cluster;
  }
};

class KMeans {
private:
  int K;  // number of clusters
  int total_values, total_points, max_iterations;
  vector<Cluster> clusters;
  
  // return ID of nearest center (uses euclidean distance)
  int getIDNearestCenter(Point point) {
    double sum = 0.0, min_dist;
    int id_cluster_center = 0;

    for (int i = 0; i < total_values; i++) {
      sum += pow(clusters[0].getCentralValue(i) - point.getValue(i), 2.0);
    }

    min_dist = sqrt(sum);

    for (int i = 1; i < K; i++) {
      double dist;
      sum = 0.0;

      for (int j = 0; j < total_values; j++) {
        sum += pow(clusters[i].getCentralValue(j) - point.getValue(j), 2.0);
      }

      dist = sqrt(sum);

      if (dist < min_dist) {
        min_dist = dist;
        id_cluster_center = i;
      }
    }

    return id_cluster_center;
  }

public:
  KMeans(int K, int total_points, int total_values, int max_iterations) {
    this->K = K;
    this->total_points = total_points;
    this->total_values = total_values;
    this->max_iterations = max_iterations;
  }

  void run(vector<Point>& points, vector<vector<Point>>& output_clusters, vector<double>& output_central_values) {
    if (K > total_points) return;

    vector<int> prohibited_indexes;

    // choose K distinct values for the centers of the clusters
    for (int i = 0; i < K; i++) {
      while (true) {
        int index_point = rand() % total_points;

        if (find(prohibited_indexes.begin(), prohibited_indexes.end(), index_point) ==
            prohibited_indexes.end()) {
          prohibited_indexes.push_back(index_point);
          points[index_point].setCluster(i);
          Cluster cluster(i, points[index_point]);
          clusters.push_back(cluster);
          break;
        }
      }
    }

    int iter = 1;

    while (true) {
      bool done = true;

      // associates each point to the nearest center
      for (int i = 0; i < total_points; i++) {
        int id_old_cluster = points[i].getCluster();
        int id_nearest_center = getIDNearestCenter(points[i]);

        if (id_old_cluster != id_nearest_center) {
          if (id_old_cluster != -1) clusters[id_old_cluster].removePoint(points[i].getID());

          points[i].setCluster(id_nearest_center);
          clusters[id_nearest_center].addPoint(points[i]);
          done = false;
        }
      }

      // recalculating the center of each cluster
      for (int i = 0; i < K; i++) {
        for (int j = 0; j < total_values; j++) {
          int total_points_cluster = clusters[i].getTotalPoints();
          double sum = 0.0;

          if (total_points_cluster > 0) {
            for (int p = 0; p < total_points_cluster; p++)
              sum += clusters[i].getPoint(p).getValue(j);
            clusters[i].setCentralValue(j, sum / total_points_cluster);
          }
        }
      }

      if (done == true || iter >= max_iterations) {
        cout << "Break in iteration " << iter << "\n\n";
        break;
      }

      iter++;
    }

    // shows elements of clusters
    for (int i = 0; i < K; i++) {  // 对于第K个cluster
      int total_points_cluster = clusters[i].getTotalPoints();

      //cout << "Cluster " << clusters[i].getID() + 1 << endl;
      for (int j = 0; j < total_points_cluster; j++) {
        // cout << "Point " << clusters[i].getPoint(j).getID() + 1 << ": ";
        // for (int p = 0; p < total_values; p++)
        //   cout << clusters[i].getPoint(j).getValue(p) << " ";   输出这个cluster的数据

        // string point_name = clusters[i].getPoint(j).getName();

        // if (point_name != "") cout << "- " << point_name;   输出name

        // cout << endl;

		output_clusters[i].push_back(clusters[i].getPoint(j));
      }

      //cout << "Cluster values: ";

      for (int j = 0; j < total_values; j++)
        //cout << clusters[i].getCentralValue(j) << " ";  // 输出这个cluster各个维度的中心值
		output_central_values[i].push_back(clusters[i].getCentralValue(j));
      //cout << "\n\n";
    }
  }
};

/*
int main() {
    srand(time(NULL));

    int total_points, total_values, K, max_iterations, has_name;

    cin >> total_points >> total_values >> K >> max_iterations >> has_name;

    vector<Point> points;
    string point_name;

    for (int i = 0; i < total_points; i++) {
      vector<double> values;

      for (int j = 0; j < total_values; j++) {
        double value;
        cin >> value;
        values.push_back(value);
      }

      if (has_name) {
        cin >> point_name;
        Point p(i, values, point_name);
        points.push_back(p);
      } else {
        Point p(i, values);
        points.push_back(p);
      }
    }

    KMeans kmeans(K, total_points, total_values, max_iterations);
    kmeans.run(points);

    return 0;
  }


*/

#endif
