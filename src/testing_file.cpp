//#include "stdafx.h"
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <list>
#include <iostream>
#include <experimental/random>

#include <random>
#include <iterator>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/aruco.hpp>
#include <algorithm>
#include <cstdlib>
#include <sstream>
#include <iostream>
#include <fstream>
#include <iterator>
#include <numeric>

using namespace std;
std::mutex mu;


template<class bidiiter>
bidiiter random_unique(bidiiter begin, bidiiter end, size_t num_random) {
    size_t left = std::distance(begin, end);
    while (num_random--) {
        bidiiter r = begin;
        std::advance(r, rand()%left);
        std::swap(*begin, *r);
        ++begin;
        --left;
    }
    return begin;
}


template<typename Iter, typename RandomGenerator>
Iter select_randomly(Iter start, Iter end, RandomGenerator& g) {
    std::uniform_int_distribution<> dis(0, std::distance(start, end) - 1);
    std::cout << "dis : " << dis(g)  << '\n';
    std::advance(start, dis(g));
    return start;
}

template<typename Iter>
Iter select_randomly(Iter start, Iter end) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    return select_randomly(start, end, gen);
}

void CallHome(string message)
{
  mu.lock();
  cout << "Thread " << this_thread::get_id() << " says " << message << endl;
  mu.unlock();
}
int main()
{
  unsigned int range = 10;
  std::string fname_ = "/home/master01/calib_data/camera/debug_960_600_102_";
  fname_.append(std::to_string(range));
  fname_.append(".txt");

  std::cout << fname_ << '\n';


  vector<std::string> foo;

  foo.push_back("1.png");
  foo.push_back("2.png");
  foo.push_back("3.png");
  foo.push_back("4.png");
  foo.push_back("5.png");
  foo.push_back("6.png");
  foo.push_back("7.png");
  foo.push_back("8.png");

//  std::string somthing;
//  for(std::vector<std::string>::const_iterator i = foo.begin(); i!=foo.end(); ++i)
//  {
//      somthing += *i + '\n';
//  }

  std::ofstream outStream(fname_);

  if(outStream){
      for(int r = 0; r < foo.size(); r++){
          std::string value = foo[r];
          outStream << value << std::endl;

      }


      outStream.close();
  }

//  output.open("test.txt",std::ios_base::trunc)
//  if(output.fail())
//    std::cerr<<"unable to open the file"std::endl;
//  output << something;
//  //after writing close file
//   output.close();

//  std::copy(foo.rbegin(), foo.rend(),
//            std::ostream_iterator<int>(fname_, "\n"));

  std::vector<std::vector<cv::Point2f>> foundPoints;

  for (int var = 0; var < 10; ++var) {
    std::vector<cv::Point2f> points;
    for (int pindex = 0; pindex < 8; ++pindex) {
      cv::Point2f point;
      point.x =  var;
      point.y = 0 + pindex - var;
      points.push_back(point);
    }

    foundPoints.push_back(points);
  }

  random_unique(foundPoints.begin(),foundPoints.end(),5);
  std::cout << foundPoints.size() << '\n';
  for(int i=0; i<10; ++i) {
      std::cout << foundPoints[i] << '\n';
  }




//  string r = *select_randomly(foo.begin(), foo.end());

//  std::cout << "r val : " << r << "\n";

////  std::set<unsigned int> indices;
//  std::vector<int> indices;
//  while (indices.size() < 5){
//      int tmp = std::experimental::randint(0,8-1);



//      indices.push_back(std::experimental::randint(0,8-1));
//  }


//  for (int i = 0; i < indices.size(); ++i) {
//    std::cout << indices[i] << "\n";
//  }


//  std::vector<unsigned int> indices(foo.size());
//  std::iota(indices.begin(), indices.end(), 0);
//  std::shuffle(indices.begin(), indices.end());



//  vector<std::string> chosen; // you don't have to use this since the chosen ones will be in the back of the vector
//  for(int i = 0; i < foo.size(); ++i) {
//    int index = std::rand_between(0, foo.size() - i - 1);
//    chosen.push_back(vec[index]);
//    swap(vec[index], vec[vec.size() - i - 1]);
//  }


//  thread t1(CallHome, "Hello from Jupiter");
//  thread t2(CallHome, "Hello from Pluto");
//  thread t3(CallHome, "Hello from Moon");
//  CallHome("Hello from Main/Earth");
//  thread t4(CallHome, "Hello from Uranus");
//  thread t5(CallHome, "Hello from Neptune");
//  t1.join();
//  t2.join();
//  t3.join();
//  t4.join();
//  t5.join();
  return 0;
}
