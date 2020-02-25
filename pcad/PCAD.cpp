#include <PCAD.h>



void PCAD::SetK(int neighborNum)
{
    K = neighborNum;
}

void PCAD::SetOPTparameters(double set_alpha, double set_beta, double set_lamda, double set_miu)
{
    alpha = set_alpha;
    beta = set_beta;
    lamda = set_lamda;
    miu = set_miu;
}

void PCAD::calculateInitialNorm()
{
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(inputcloud);
	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 3cm
	ne.setKSearch(K);
	// Compute the features
	ne.compute(*initial_normal);
	//std::cout << initial_normal->points[0].normal_x << initial_normal->points[0].normal_y << initial_normal->points[0].normal_z << std::endl;
}


void PCAD::Denoise()
{	
	//calculateInitialNorm();
    for (int i=0;i<inputcloud->points.size();i++)//for loop? should we input single index and point to Denoise() and for loop in main()?
    //for (int i=0;i<1;i++)
	{
		if(i%10000==0)printf("Denoising point %d\n",i);
        k_nbrs=get_k_nbrs(i);//get k_neighbours
		//std::cout<<k_nbrs[0]<<std::endl<<k_nbrs[1]<<std::endl<<k_nbrs[2]<<std::endl;
		//printf("Point %d k_nbrs extract finished\n", i);
        nc.setparam(k_nbrs,searchpoint,alpha,beta,lamda,miu);
		//printf("Point %d set param finished\n", i);
        //nc_d.setparam(k_nbrs,p,param);
		//double initial_fai = asin(initial_normal->points[i].normal_z);
		//double initial_theta = asin(initial_normal->points[i].normal_y/cos(initial_fai));
		//double initial_fai = 1.57;
		//double initial_theta = 0.5;
        if (i==0)starting_point = {inputcloud->points[i].x,inputcloud->points[i].y,inputcloud->points[i].z,1.57,0.5}; // Start with a valid point inside the constraint box.
		else starting_point = {inputcloud->points[i].x,inputcloud->points[i].y,inputcloud->points[i].z,last_fai,last_theta};//starting_point = { inputcloud->points[i].x,inputcloud->points[i].y,inputcloud->points[i].z,initial_fai,initial_theta };
		//column_vector mm = dlib::derivative(nc);
		//std::cout <<"initial angles"<<initial_fai<<"  "<<initial_theta  << std::endl;
        //dlib::find_min(dlib::lbfgs_search_strategy(10),
        //                     dlib::objective_delta_stop_strategy(1e-5),
        //                     nc, dlib::derivative(nc), starting_point,-1);

		dlib::find_min_using_approximate_derivatives(dlib::lbfgs_search_strategy(10),
			dlib::objective_delta_stop_strategy(1e-4),
			nc, starting_point, -1);
		//column_vector test_x = { 1,1,1,1,1 };
		//std::cout << nc(starting_point) << std::endl;
		//column_vector low = { -(std::numeric_limits<double>::infinity()),-(std::numeric_limits<double>::infinity()),-(std::numeric_limits<double>::infinity()),-(std::numeric_limits<double>::infinity()),-(std::numeric_limits<double>::infinity()) };
		//column_vector high = { { (std::numeric_limits<double>::infinity()),(std::numeric_limits<double>::infinity()),(std::numeric_limits<double>::infinity()),(std::numeric_limits<double>::infinity()),(std::numeric_limits<double>::infinity()) } };
		
		//dlib::find_min_box_constrained(dlib::lbfgs_search_strategy(10),
		//	dlib::objective_delta_stop_strategy(1e-7),
		//	nc, dlib::derivative(nc), starting_point, low ,high);
		//printf("Point %d find_min finished\n", i);
        filter_point(starting_point,i);//filter the point[i] according to optimum result
		//std::cout<<starting_point(0)<<" "<<starting_point(1)<<" "<<starting_point(2)<<" "<<starting_point(3)<<" "<<starting_point(4)<<" "<<std::endl;
		last_fai-=int(starting_point(3)*0.5/PI+0.5)*2*PI;
		last_theta-=int(starting_point(4)*0.5/PI+0.5)*2*PI;
	}
}

void PCAD::filter_point(const column_vector&m,int i)
{
    //filter the position of point[i] according to column_vecotr m
    cloud_normal->points[i].normal_x=cos(m(3))*cos(m(4));
    cloud_normal->points[i].normal_y=cos(m(3))*sin(m(4));
    cloud_normal->points[i].normal_z=sin(m(3));
    filter_cloud->points[i].x=m(0);
    filter_cloud->points[i].y=m(1);
    filter_cloud->points[i].z=m(2);
}

void PCAD::InputPointcloud(const char *input_name)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normal_tmp(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;
    reader.read<pcl::PointXYZ> (input_name, *inputcloud_tmp);
	inputcloud = inputcloud_tmp;
	pcl::PointCloud<pcl::Normal>::Ptr initial_norm_temp(new pcl::PointCloud<pcl::Normal>);
	initial_normal = initial_norm_temp;

	filter_cloud = filter_cloud_tmp;
    filter_cloud->width=inputcloud->points.size();
    filter_cloud->height=1;
    filter_cloud->points.resize(filter_cloud->width *filter_cloud->height );
	cloud_normal = cloud_normal_tmp;
	cloud_normal->width = inputcloud->points.size();
	cloud_normal->height = 1;
	cloud_normal->points.resize(cloud_normal->width * cloud_normal->height);
	kdtree.setInputCloud(inputcloud);
	
}
void PCAD::InputPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr inputprt)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normal_tmp(new pcl::PointCloud<pcl::Normal>);
	inputcloud = inputprt;
	pcl::PointCloud<pcl::Normal>::Ptr initial_norm_temp(new pcl::PointCloud<pcl::Normal>);
	initial_normal = initial_norm_temp;

	filter_cloud = filter_cloud_tmp;
    filter_cloud->width=inputcloud->points.size();
    filter_cloud->height=1;
    filter_cloud->points.resize(filter_cloud->width *filter_cloud->height );
	cloud_normal = cloud_normal_tmp;
	cloud_normal->width = inputcloud->points.size();
	cloud_normal->height = 1;
	cloud_normal->points.resize(cloud_normal->width * cloud_normal->height);
	
}

void PCAD::OutputPointcloud(const char *output_name)
{
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> (output_name, *filter_cloud, false);
}

std::vector<Eigen::Vector3d> PCAD::get_k_nbrs(int index)
{
	k_nbrs.resize(K+1);
    std::vector<float> pointNKNSquaredDistance(K+1);
    std::vector<int>  k_indices(K+1);
	Eigen::Vector3d tmp(inputcloud->points[index].x, inputcloud->points[index].y, inputcloud->points[index].z);
	
	searchpoint= tmp;
    kdtree.nearestKSearch(index, K+1, k_indices, pointNKNSquaredDistance);

	//std::cout<<"K"<<K<<std::endl;
    for (int i=0;i<K+1;i++)
    {
		//std::cout<<"k_indice[i]  "<<k_indices[i]<<std::endl;
        Eigen::Vector3d tmp(inputcloud->points[k_indices[i]].x,inputcloud->points[k_indices[i]].y,inputcloud->points[k_indices[i]].z);
		k_nbrs[i]=tmp;
		//std::cout<<"k_nbrs[i]"<<k_nbrs[i]<<std::endl;
    }
	//std::cout<<"k_nbr size "<<k_nbrs.size()<<std::endl;
	k_nbrs.erase(k_nbrs.begin());
	//std::cout<<"k_nbr size "<<k_nbrs.size()<<std::endl;
	//std::cout<<k_nbrs[0]<<std::endl<<k_nbrs[1]<<std::endl<<k_nbrs[2]<<std::endl;
    return k_nbrs;
}



