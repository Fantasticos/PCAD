#include <iostream>
#include <PCAD.h>

int main(int argc, const char* argv[])
{

    using namespace std::chrono;

    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    if (argc < 2)
    {
        printf("Usage: <input_name.pcd> <output_name.pcd> K alpha beta lamda miu");
        return 0;
    }
    printf("Input .pcd file name: %s\n", argv[1]);
    printf("Output .pcd file name: %s\n", argv[2]);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    filter_cloud->width=4;
    filter_cloud->height=1;
    filter_cloud->points.resize(filter_cloud->width *filter_cloud->height );
    filter_cloud->points[0].x=0;
    filter_cloud->points[0].y=0;
    filter_cloud->points[0].z=1;

    filter_cloud->points[1].x=1;
    filter_cloud->points[1].y=0;
    filter_cloud->points[1].z=0;

    filter_cloud->points[2].x=0;
    filter_cloud->points[2].y=1;
    filter_cloud->points[2].z=0;

    filter_cloud->points[3].x=0;
    filter_cloud->points[3].y=-1;
    filter_cloud->points[3].z=0;

	int K = std::stoi(argv[3]);
	//double alpha = std::stod(argv[4]);
	//double beta = std::stod(argv[5]);
	//double lamda = std::stod(argv[6]);
	//double miu = std::stod(argv[7]);
	double alpha =0.05;
	double beta = 0.5;
	double lamda = 0.5;
	double miu = 0.0001;

    printf("K = %d\n", std::stoi(argv[3]));
    printf("alpha = %lf\n", alpha);
    printf("beta = %lf\n", beta);
    printf("lamda = %lf\n", lamda);
    printf("miu = %lf\n", miu);

    PCAD pcad;
	printf("class define finished\n");
    pcad.InputPointcloud(argv[1]); 
    //pcad.InputPointcloud(filter_cloud);
	printf("input finished\n");
    pcad.SetK(K);
	printf("set finished\n");
    pcad.SetOPTparameters(alpha, beta,lamda,miu);
	printf("setopt finished\n");
	pcad.Denoise();
	printf("denoise finished\n");
    pcad.OutputPointcloud(argv[2]);


  high_resolution_clock::time_point t2 = high_resolution_clock::now();

  duration<double, std::milli> time_span = t2 - t1;

  std::cout << "Time Span" << time_span.count()/1000 << " seconds.";
  std::cout << std::endl;
    
	return 0;
}