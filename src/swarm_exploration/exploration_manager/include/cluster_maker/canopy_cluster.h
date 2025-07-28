// CPP IMPLEMENTATION OF CANOPY CLUSTERING ALGORITHM
#ifndef _CANOPY_CLUSTER_H_
#define _CANOPY_CLUSTER_H_

#include <bits/stdc++.h>
#include <point.h>

using namespace std;
int has_name;

class Cluster {
private:
  int id_cluster;
  vector<Point> points;

public:
  void addPoint(Point point) {
    points.push_back(point);
  }
  int getID() {
    return id_cluster;
  }
  void setID(int id) {
    this->id_cluster = id;
  }

  vector<Point> getPoints() {
    return points;
  }
};

class Canopy

{
private:
  vector<Point> points;
  int total_points, total_values;
  double t1, t2;
  vector<Cluster> clusters;

public:
  Canopy(int total_points, int total_values, vector<Point> points, double t1, double t2) {
    this->total_points = total_points;
    this->total_values = total_values;
    this->t1 = t1;
    this->t2 = t2;
    this->points = points;
  }
  double findEuclidDistance(Point p1, Point p2) {

    double sum = 0.0;
    for (int i = 0; i < total_values; ++i) {
      sum += pow(p1.getValue(i) - p2.getValue(i), 2.0);
    }
    return sqrt(sum);
  }

  void removePoint(Point p) {
    for (int i = 0; i < points.size(); i++) {
      if (p.getID() == points[i].getID()) {
        points.erase(points.begin() + i);
        break;
      }
    }
  }

  void go() {
    srand((int)time(NULL));

    int k = 1;
    while (points.size() > 0) {
      int Index = rand() % points.size();
      // cout<<Index<<endl;
      Point temp_point = points[Index];
      Cluster cluster;
      cluster.setID(k);
      // if distance is less than t1 a new cluster is formed
      for (int i = 0; i < points.size(); i++) {
        double distance = findEuclidDistance(temp_point, points[i]);

        if (distance < t1) {

          cluster.addPoint(points[i]);
        }
      }

      vector<Point> clusterPoints = cluster.getPoints();
      // In the above formed cluster if distance is less than t2 we remove it from main set
      for (int j = 0; j < clusterPoints.size(); j++) {
        Point p = clusterPoints[j];
        double distance = findEuclidDistance(temp_point, p);
        if (distance < t2) removePoint(p);
      }

      clusters.push_back(cluster);
      k++;
    }
  }

  void print(vector<vector<Point>>& output_clusters) {

    for (int i = 0; i < clusters.size(); i++) {
      vector<Point> clusterPoints;
      clusterPoints = clusters[i].getPoints();
      // cout<<endl;
      // cout<<"cluster "<<clusters[i].getID()<<endl;
      // cout<<endl;

      for (int j = 0; j < clusterPoints.size(); j++) {
        // Point p = clusterPoints[j];

        // for(int h=0;h<total_values;h++)
        // {
        //     cout<<p.getValue(h)<<" ";
        // }

        // if(has_name)
        //     cout<<p.getName();
        // cout<<endl;
        output_clusters.push_back(clusterPoints[j]);
      }
    }
  }
};

/*
int main(int arg,char*argv[])
{

    // if(arg<2)
    // {
    //     cout<<"please enter input file name as argument"<<endl;
    //     return 0;
    // }
    // FILE* input = freopen(argv[1],"r",stdin);


    // int total_points, total_values;
   // double t1,t2;

    //cin >>total_points>> total_values >>t1>>t2>>has_name;
    //please read readme.txt to understand inputs



    vector<Point> points;
    string point_name;

    for(int i = 0; i < total_points; i++)
    {
        vector<double> values;

        for(int j = 0; j<total_values;j++)
        {
            double value;
            cin >> value;
            values.push_back(value);
        }

        if(has_name)
        {
            cin >> point_name;
            Point p(i, values, point_name);
            points.push_back(p);
        }
        else
        {
            Point p(i, values,"");
            points.push_back(p);
        }
    }
    fclose(input);

    if(t2>=t1){
       cout<<"To proceed t2 should be less than t1"<<endl;
       return 0;
    }
    Canopy canopy(total_points, total_values, points);
    canopy.go();
    canopy.print();


    return 0;
}
*/

#endif
