#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/surface/gp3.h>

int
main (int argc, char** argv)
{
    std::string input, output;
    if(argc < 3)
    {
        std::cerr << "Not enoght arguments" << '\n';
        return(-1);
    }
    else
    {
        input = std::string(argv[1]);
        output = std::string(argv[2]);
    }

    std::cout << argc << std::endl;

    for (int i = 1; i < argc; ++i) {
        std::cout << argv[i] << std::endl;
    }

    std::cout << "Reading data...";
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 cloud_blob;
    pcl::io::loadPCDFile (input, cloud_blob);
    pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
    std::cout << "Done" << std::endl;
    //* the data should be available in cloud


    std::cout << "Estimating normals...";
    // Normal estimation*
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    //n.setKSearch (20);
    // Use all neighbors in a sphere of radius 3cm
    n.setRadiusSearch (0.03);
    n.compute (*normals);
    std::cout << "Done" << std::endl;
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    std::cout << "Concatenating normals with coordinates...";
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    std::cout << "Done" << std::endl;
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    std::cout << "GreedyProjectionTriangulation...";
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::PolygonMesh triangles;
    // pcl::MarchingCubesRBF<pcl::PointNormal> mc;
    // Get result
    // mc.setInputCloud (cloud_with_normals);
    // mc.setSearchMethod (tree2);
    // mc.reconstruct (triangles);

    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;


    // // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (0.25);
    //
    // // Set typical values for the parameters
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (300);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);
    std::cout << "Done" << std::endl;
    // Additional vertex information
    // std::vector<int> parts = mc.getPartIDs();
    // std::vector<int> states = mc.getPointStates();

    // Finish
    pcl::io::saveOBJFile(output, triangles);
    return (0);
}
