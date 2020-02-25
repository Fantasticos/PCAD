#ifndef PCAD_h
#define PCAD_h
#include <iostream>
#include <thread>
#include <vector>
#include <ctime>
#include <ratio>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <dlib/optimization.h>
#include <dlib/global_optimization.h>
#include <pcl/features/normal_3d.h>
#define PI 3.1415926535897932

typedef dlib::matrix <double, 0, 1> column_vector;

//Functor class
class normalcost
{
public:
	normalcost() { }
	void setparam(std::vector<Eigen::Vector3d> thek_nbrs, Eigen::Vector3d thep, double a,double b,double l,double m)
	{ 
	k_nbrs=thek_nbrs;
	alpha=a;
	beta =b;
	lamda=l;
	miu=m;
	p=thep;
	//std::cout << k_nbrs.size() << std::endl;
	}

	double operator() (const column_vector& m) const
	{
		Eigen::Vector3d pp;//estimated position
		pp(0)=m(0);
		pp(1)=m(1);
		pp(2)=m(2);
		Eigen::Vector3d nn;//estimated normal vector
		nn(0)=cos(m(3))*cos(m(4));
		nn(1)=cos(m(3))*sin(m(4));
		nn(2)=sin(m(3));
		//std::cout << " pp1 "<<pp(0)<<" pp2 "<<pp(1)<<" pp3 "<<pp(2)<<std::endl;
		//std::cout << " nn1 "<<nn(0)<<" nn2 "<<nn(1)<<" nn3 "<<nn(2)<<std::endl;
		//std::cout << " m3 "<<m(3)<<" m4 "<<m(4)<<std::endl;
		double part1=0;double part2 = 0;double part3=0;
		double normal_error=0;
		part1=0.5*exp(alpha*(p-pp).squaredNorm());
		//std::cout << "estimated point"  << "is" <<std::endl<< pp << std::endl;
		//std::cout << "estimated normal" << "is" <<std::endl<< nn << std::endl;
		for (int i=0;i<k_nbrs.size();i++)
		{
			normal_error=((k_nbrs[i]-pp).dot(nn))* ((k_nbrs[i] - pp).dot(nn));
			//std::cout <<"normal error for neighbour "<<i<<"is"<< normal_error << std::endl;
			part2+=exp(-beta*(k_nbrs[i]-p).squaredNorm())*exp(-lamda*normal_error)*normal_error;
			//part3+=1/((k_nbrs[i]-pp).squaredNorm()+0.000001);
		}
		//part3=part3*0.5*miu;
		//std::cout << part1 * part2 + part3<<std::endl;
		//	std::cout << " part1 "<<part1<<" part2 "<<part2<<std::endl;

		return part1 * part2;
		//return part1 * part2 + part3;
	}
private:
	std::vector<Eigen::Vector3d> k_nbrs;//K vector with vector3d representing position of K neighbours
	Eigen::Vector3d p;//current point position
	double alpha;
	double beta;
	double lamda;
	double miu;
	
};


class PCAD
{
public:
	PCAD() {};
	void InputPointcloud(const char* input_name);
	void SetK(int neighborNum);
	void SetOPTparameters(double set_alpha, double set_beta, double set_lamda, double set_miu);
	void calculateInitialNorm();
	void Denoise();
	void filter_point(const column_vector& m, int i);
	void OutputPointcloud(const char* output_name);
	void InputPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr);


public:
	int K;
	double alpha;
	double beta;
	double lamda;
	double miu;
	//member variables, all length 5, format {x,y,z,fai,theta}
	column_vector starting_point;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normal;
	normalcost nc;
	//normalcost_derivative nc_d;
	//Eigen::Vector4d param;

private:
	std::vector<Eigen::Vector3d> get_k_nbrs(int index);
	double last_fai;
	double last_theta;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
private:
	pcl::PointCloud<pcl::Normal>::Ptr initial_normal;
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud;
	std::vector<Eigen::Vector3d> k_nbrs;//K vector with vector3d representing position of K neighbours
	Eigen::Vector3d searchpoint;//current point position

};




#endif /* PCAD_h */
