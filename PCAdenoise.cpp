typedef matrix<double, 0, 1> column_vector;

// ----------------------------------------------------------------------------------------

class normalcost
{
public:
	functor() { }
	void setparam(vector<Eigen::Vector3d> thek_nbrs, Eigen::Vector3d thep, Eigen::Vector4d theparam)
	{ 
	k_nbrs=thek_nbrs;
	alpha=theparam(0);
	beta = =theparam(1);
	lamda==theparam(2);
	miu==theparam(3);
	p=thep;
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
		double part1=0;double part2 = 0;double part3=0;
		double normal_error=0;
		part1=0.5*exp(alpha*(p-pp).squaredNorm());
		for (int i=0;i<k_nbrs.size();i++)
		{
			normal_error=((k_nbrs[i]-pp).dot(nn))^2;
			part2+=exp(-beta*(k_nbrs[i]-p).squaredNorm())*exp(-lamda*normal_error)*normal_error;
			part3+=1/(k_nbrs[i]-pp).squaredNorm();
		}
		part3=part3*0.5*miu;
		return part1*part2+part3
	}
private:
	vector<Eigen::Vector3d> k_nbrs;//K vector with vector3d representing position of K neighbours
	Eigen::Vector3d p;//current point position
	double alpha;
	double beta;
	double lamda;
	double miu;
	
};
/*
class normalcost_derivative
{
public:
	functor() { }
	void setparam(vector<Eigen::Vector3d> thek_nbrs, Eigen::Vector3d thep, Eigen::Vector4d theparam)
	{ 
	k_nbrs=thek_nbrs;
	alpha=theparam(0);
	beta = =theparam(1);
	lamda==theparam(2);
	miu==theparam(3);
	p=thep;
	}

	double operator() (const column_vector& m) const
	{
		column_vector res(5);
		double cost_d=f_d(k_nbrs,p,params,m)
		return cost_d
	}
private:
	vector<Eigen::Vector3d> k_nbrs;//K vector with vector3d representing position of K neighbours
	Eigen::Vector3d p;//current point position
	double alpha;
	double beta;
	double lamda;
	double miu;
	
};
*/



void PCAD::filter_point(const column_vector&m,int i)
{
	//filter the position of point[i] according to column_vecotr m
	cloud_normal.x=cos(m(3))*cos(m(4));
	cloud_normal.y=cos(m(3))*sin(m(4));
	cloud_normal.z=sin(m(3));
	filter_cloud->points[i].x=m(0);
	filter_cloud->points[i].y=m(1);
	filter_cloud->points[i].z=m(2);
}

//member variables, all length 5, format {x,y,z,fai,theta}
column_vector starting_point;
column_vector x_low;
column_vector x_up;
pcl::normal::prt cloud_normal;
normalcost nc;
//normalcost_derivative nc_d;
Eigen::Vector4d param;



void PCAD::denoise()
{
	for (int i=0;i<inputcloud->points.size();i++)
	{
		k_nbrs=get_k_nbrs(i);//get k_neighbours
		nc.setparam(k_nbrs,searchpoint,param);
		//nc_d.setparam(k_nbrs,p,param);
		starting_point = {}; // Start with a valid point inside the constraint box.
		
		find_min(lbfgs_search_strategy(10),  
                             objective_delta_stop_strategy(1e-9),  
                             nc, derivative(nc), starting_point, x_low, x_up);
		filter_point(starting_point,i);//filter the point[i] according to optimum result
	}
}