#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <string>
#include <fstream>
#include <stdlib.h> 
#include <vector>
#include <sstream>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <boost/format.hpp>
#include <boost/thread/mutex.hpp>
#include <queue>
#include <stack>
#include <TriangleBoxIntersection.h>
#include <stdint.h>


enum cell_state {non_initialized=0, empty=1, occupied=2, outlet=3, edge=4};

struct Point{
    float x; float y; float z;
    Point(){}
    Point(float x, float y, float z){
        this->x = x; this->y =y; this->z=z;
    }
};
struct Triangle{
    Point p1; Point p2; Point p3;
    Triangle(){}
    Triangle(Point p1, Point p2, Point p3){
        this->p1=p1; this->p2=p2; this->p3=p3;
    }
    Point& operator[](int i){
        if(i==0)
            return p1;
        else if (i==1)
            return p2;
        else if(i==2)
            return p3;
        else{
            std::cout<<"Indexing error when accessing the points in triangle! Index must be >= 2";
            return p1;
        }
    }
};

//dimensions of the enviroment [m]
float env_min_x; 
float env_min_y; 
float env_min_z;
float env_max_x; 
float env_max_y; 
float env_max_z;
float roundFactor;
//length of the sides of the cell [m]
float cell_size;
float floor_height;


std::vector<std::vector<std::vector<int> > > env;

bool compare_cell(int x, int y, int z, cell_state value){
    if(x<0 || x>=env.size() ||
        y<0 || y>=env[0].size() ||
        z<0 || z>=env[0][0].size()){
            return false;
    }
    else{
        return env[x][y][z]==value;
    }
}

void changeWorldFile(std::string filename){
    std::ifstream input(filename);
    std::stringstream ss;
    std::string line;
    while(getline(input, line)){
        if(line.substr(0,8)=="floorMap"){
            //ignore the floorMap bit, we are replacing it entirely
            while(getline(input, line) && line!=")"){}

            ss<< 
            "floorMap                     # load an environment bitmap\n"<<
            "(\n"<<
                "  name \"SimulatedMap\"\n"<< 
                "  bitmap \"../../occupancy.pgm\"\n"<<
                "  size ["<<(env_max_x-env_min_x)<<" "<<(env_max_y-env_min_y)<<" "<<(env_max_z-env_min_z) <<"]           #m \n"<< 
                "  pose ["<<(env_max_x-env_min_x)/2+env_min_x<<" "<<(env_max_y-env_min_y)/2+env_min_y<<" "<<floor_height<<" 0]    #Coordinates (m) of the Center of the image_map\n"<<
            ")\n";
        }
        else{
            ss<<line<<"\n";
        }
    }
    input.close();
    std::ofstream out(filename);
    out<<ss.rdbuf();
    out.close();
}

void printMap(std::string filename, int scale){
    std::ofstream outfile(filename.c_str());
    outfile << "P2\n"
            << scale *  env[0].size() << " " << scale * env.size() << "\n" <<"1\n";
    //things are repeated to scale them up (the image is too small!)

    int height = (floor_height-env_min_z)/cell_size; //a xy slice of the 3D environment is used as a geometric map for navigation
    
    for (int row = env.size()-1; row >= 0; row--)
    {
        for (int j = 0; j < scale; j++)
        {
            for (int col = 0; col <env[0].size() ; col++)
            {
                for (int i = 0; i < scale; i++)
                {
                    outfile << (env[row][col][height] == cell_state::empty? 1 : 0) << " ";
                }
            }
            outfile << "\n";
        }
    }
    outfile.close();

}

void printEnv(std::string filename, int scale)
{
    std::ofstream outfile(filename.c_str());
    
    outfile <<  "#env_min(m) " << env_min_x << " " << env_min_y << " " << env_min_z << "\n";
    outfile <<  "#env_max(m) " << env_max_x << " " << env_max_y << " " << env_max_z << "\n";
    outfile <<  "#num_cells " << env[0].size() << " " << env.size() << " " << env[0][0].size() << "\n";
    outfile <<  "#cell_size(m) " << cell_size << "\n";
    //things are repeated to scale them up (the image is too small!)
    for (int height = 0; height < env[0][0].size(); height++)
    {
        for (int col = 0; col <env[0].size(); col++)
        {
            for (int j = 0; j < scale; j++)
            {
                for (int row = 0; row <env.size(); row++)
                {
                    for (int i = 0; i < scale; i++)
                    {
                        outfile << (env[row][col][height]==cell_state::empty? 0 :
                                (env[row][col][height]==cell_state::outlet? 2 :
                                1))
                                << " ";
                    }
                }
                outfile << "\n";
            }
        }
        outfile << ";\n";
    }
    outfile.close();
}

void printWind(std::vector<double> U,
                std::vector<double> V,
                std::vector<double> W, std::string filename){
    
    std::ofstream fileU(boost::str(boost::format("%s_U") % filename).c_str());
    std::ofstream fileV(boost::str(boost::format("%s_V") % filename).c_str());
    std::ofstream fileW(boost::str(boost::format("%s_W") % filename).c_str());
    
    //this code is a header to let the filament_simulator know the file is in binary
    int code=999;

    fileU.write((char*) &code, sizeof(int));
    fileV.write((char*) &code, sizeof(int));
    fileW.write((char*) &code, sizeof(int));

    fileU.write((char*) U.data(), sizeof(double) * U.size());
    fileV.write((char*) V.data(), sizeof(double) * V.size());
    fileW.write((char*) W.data(), sizeof(double) * W.size());

    fileU.close();
    fileV.close();
    fileW.close();
}

void printYaml(std::string output){
    std::ofstream yaml(boost::str(boost::format("%s/occupancy.yaml") % output.c_str()));
    yaml << "image: occupancy.pgm\n" 
        << "resolution: " << cell_size/10 
        << "\norigin: [" << env_min_x << ", " << env_min_y << ", " << 0 << "]\n"
        << "occupied_thresh: 0.9\n" 
        << "free_thresh: 0.1\n" 
        << "negate: 0";
    yaml.close();
}

float min_val(float x, float y, float z) {

    float min =x;
    if (y < min)
        min=y;
    if(z < min)
        min=z;

    return min;
}
float max_val(float x, float y, float z) {

    float max= x;
    if (y > max)
        max=y;
    if(z > max)
        max=z;

    return max;
}
bool eq(float x, float y){
    return std::abs(x-y)<0.01;
}

std::vector<Eigen::Vector3d> cubePoints(const Eigen::Vector3d &query_point){
    std::vector<Eigen::Vector3d> points;
    points.push_back(query_point);
    points.push_back(Eigen::Vector3d(query_point.x()-cell_size/2,
                                            query_point.y()-cell_size/2,
                                            query_point.z()-cell_size/2));
    points.push_back(Eigen::Vector3d(query_point.x()-cell_size/2,
                                            query_point.y()-cell_size/2,
                                            query_point.z()+cell_size/2));
    points.push_back(Eigen::Vector3d(query_point.x()-cell_size/2,
                                            query_point.y()+cell_size/2,
                                            query_point.z()-cell_size/2));
    points.push_back(Eigen::Vector3d(query_point.x()-cell_size/2,
                                            query_point.y()+cell_size/2,
                                            query_point.z()+cell_size/2));
    points.push_back(Eigen::Vector3d(query_point.x()+cell_size/2,
                                            query_point.y()-cell_size/2,
                                            query_point.z()-cell_size/2));
    points.push_back(Eigen::Vector3d(query_point.x()+cell_size/2,
                                            query_point.y()-cell_size/2,
                                            query_point.z()+cell_size/2));
    points.push_back(Eigen::Vector3d(query_point.x()+cell_size/2,
                                            query_point.y()+cell_size/2,
                                            query_point.z()-cell_size/2));
    points.push_back(Eigen::Vector3d(query_point.x()+cell_size/2,
                                            query_point.y()+cell_size/2,
                                            query_point.z()+cell_size/2));
    return points;
}

bool pointInTriangle(const Eigen::Vector3d& query_point,
                     const Eigen::Vector3d& triangle_vertex_0,
                     const Eigen::Vector3d& triangle_vertex_1,
                     const Eigen::Vector3d& triangle_vertex_2)
{
    // u=P2−P1
    Eigen::Vector3d u = triangle_vertex_1 - triangle_vertex_0;
    // v=P3−P1
    Eigen::Vector3d v = triangle_vertex_2 - triangle_vertex_0;
    // n=u×v
    Eigen::Vector3d n = u.cross(v);
    bool anyProyectionInTriangle=false;
    std::vector<Eigen::Vector3d> cube= cubePoints(query_point);
    for(const Eigen::Vector3d &vec : cube){
        // w=P−P1
        Eigen::Vector3d w = vec - triangle_vertex_0;
        // Barycentric coordinates of the projection P′of P onto T:
        // γ=[(u×w)⋅n]/n²
        float gamma = u.cross(w).dot(n) / n.dot(n);
        // β=[(w×v)⋅n]/n²
        float beta = w.cross(v).dot(n) / n.dot(n);
        float alpha = 1 - gamma - beta;
        // The point P′ lies inside T if:
        bool proyectionInTriangle= ((0 <= alpha) && (alpha <= 1) &&
                (0 <= beta)  && (beta  <= 1) &&
                (0 <= gamma) && (gamma <= 1));
        anyProyectionInTriangle=anyProyectionInTriangle||proyectionInTriangle;
    }

    n.normalize();
    
    //we consider that the triangle goes through the cell if the proyection of the center 
    //is inside the triangle AND the plane of the triangle intersects the cube of the cell
    
    return anyProyectionInTriangle;
}

bool parallel (const Point &vec){
    return (eq(vec.y,0)
                &&eq(vec.z,0))||
           (eq(vec.x,0)
                &&eq(vec.z,0))||
           (eq(vec.x,0)
                &&eq(vec.y,0));
}

void occupy(std::vector<Triangle> &triangles,
const std::vector<Point> &normals,
            cell_state value_to_write){

    std::cout<<"Processing the mesh...\n0%\n";
    int numberOfProcessedTriangles=0; //for logging, doesn't actually do anything
    boost::mutex mtx;
    //Let's occupy the enviroment!
    #pragma omp parallel for
    for(int i= 0;i<triangles.size();i++){
        //We try to find all the cells that some triangle goes through
        int x1 = roundf((triangles[i].p1.x-env_min_x)*(roundFactor))/(cell_size*(roundFactor));
        int y1 = roundf((triangles[i].p1.y-env_min_y)*(roundFactor))/(cell_size*(roundFactor));
        int z1 = roundf((triangles[i].p1.z-env_min_z)*(roundFactor))/(cell_size*(roundFactor));
        int x2 = roundf((triangles[i].p2.x-env_min_x)*(roundFactor))/(cell_size*(roundFactor));
        int y2 = roundf((triangles[i].p2.y-env_min_y)*(roundFactor))/(cell_size*(roundFactor));
        int z2 = roundf((triangles[i].p2.z-env_min_z)*(roundFactor))/(cell_size*(roundFactor));
        int x3 = roundf((triangles[i].p3.x-env_min_x)*(roundFactor))/(cell_size*(roundFactor));
        int y3 = roundf((triangles[i].p3.y-env_min_y)*(roundFactor))/(cell_size*(roundFactor));
        int z3 = roundf((triangles[i].p3.z-env_min_z)*(roundFactor))/(cell_size*(roundFactor));

        int min_x = min_val(x1,x2,x3);
        int min_y = min_val(y1,y2,y3);
        int min_z = min_val(z1,z2,z3);

        int max_x = max_val(x1,x2,x3);
        int max_y = max_val(y1,y2,y3);
        int max_z = max_val(z1,z2,z3);

        //is the triangle right at the boundary between two cells (in any axis)?
        bool xLimit = eq(std::fmod(max_val(triangles[i][0].x,triangles[i][1].x,triangles[i][2].x)-env_min_x, cell_size),0)
            ||eq(std::fmod(max_val(triangles[i][0].x,triangles[i][1].x,triangles[i][2].x)-env_min_x, cell_size),cell_size);

        bool yLimit = eq(std::fmod(max_val(triangles[i][0].y,triangles[i][1].y,triangles[i][2].y)-env_min_y, cell_size),0)
            ||eq(std::fmod(max_val(triangles[i][0].y,triangles[i][1].y,triangles[i][2].y)-env_min_y, cell_size),cell_size);
            
        bool zLimit = eq(std::fmod(max_val(triangles[i][0].z,triangles[i][1].z,triangles[i][2].z)-env_min_z, cell_size),0)
            ||eq(std::fmod(max_val(triangles[i][0].z,triangles[i][1].z,triangles[i][2].z)-env_min_z, cell_size),cell_size);

        bool isParallel =parallel(normals[i]);
        for (int row = min_x; row <= max_x && row < env[0].size(); row++)
        {
            for (int col = min_y; col <= max_y && col < env.size(); col++)
            {
                for (int height = min_z; height <= max_z && height < env[0][0].size(); height++)
                {
                    //check if the triangle goes through this cell
                    //special case for triangles that are parallel to the coordinate axes because the discretization can cause
                    //problems if they fall right on the boundary of two cells
                    if (
                        (isParallel && pointInTriangle(Eigen::Vector3d(row * cell_size + env_min_x+cell_size/2,
                                                        col * cell_size + env_min_y+cell_size/2,
                                                        height * cell_size + env_min_z+cell_size/2),
                                        Eigen::Vector3d(triangles[i][0].x, triangles[i][0].y, triangles[i][0].z),
                                        Eigen::Vector3d(triangles[i][1].x, triangles[i][1].y, triangles[i][1].z),
                                        Eigen::Vector3d(triangles[i][2].x, triangles[i][2].y, triangles[i][2].z)))
                    || 
                    triBoxOverlap(
                            Eigen::Vector3d(row * cell_size + env_min_x+cell_size/2,
                                col * cell_size + env_min_y+cell_size/2,
                                height * cell_size + env_min_z+cell_size/2),
                            Eigen::Vector3d(cell_size/2, cell_size/2, cell_size/2),
                            Eigen::Vector3d(triangles[i][0].x, triangles[i][0].y, triangles[i][0].z),
                                    Eigen::Vector3d(triangles[i][1].x, triangles[i][1].y, triangles[i][1].z),
                                    Eigen::Vector3d(triangles[i][2].x, triangles[i][2].y, triangles[i][2].z)))
                    {
                        mtx.lock();
                        env[col][row][height] = value_to_write;
                        if(value_to_write==cell_state::occupied){
                            //if the "limit" flags are activated, AND we are on the offending cells, 
                            //AND the cell has not previously marked as normally occupied by a different triangle
                            //AND the cells are not on the very limit of the environment, mark the cell as "edge" for later cleanup
                            bool limitOfproblematicTriangle=(xLimit&&row==max_x)||
                                (yLimit&&col==max_y)||
                                (zLimit&&height==max_z);

                            bool endOfTheEnvironment = (col>0 || col<env.size() ||
                                row>0 || row<env[0].size() ||
                                height>0 ||  height<env[0][0].size());

                            if( !endOfTheEnvironment &&
                                limitOfproblematicTriangle && 
                                env[col][row][height]!=cell_state::occupied){
                                    env[col][row][height]=cell_state::edge;
                            }
                        }
                        mtx.unlock();

                    }
                }
            }
        }

        //log progress
        if(i>numberOfProcessedTriangles+triangles.size()/10){
            mtx.lock();
            std::cout<<(100*i)/triangles.size()<<"%\n";
            numberOfProcessedTriangles=i;
            mtx.unlock();
        }
    }
}  

void parse(std::string filename, cell_state value_to_write){
    
    bool ascii = false;
    if (FILE *file = fopen(filename.c_str(), "r"))
    {
        //File exists!, keep going!
        char buffer[6];
        fgets(buffer, 6, file);
        if(std::string(buffer).find("solid")!=std::string::npos)
            ascii=true;
        fclose(file);
    }else{
        std::cout<< "File " << filename << " does not exist\n";
        return;
    }
    
    std::vector<Triangle> triangles;
    std::vector<Point> normals;

    if(ascii){
        //first, we count how many triangles there are (we need to do this before reading the data 
        // to create a vector of the right size)
        std::ifstream countfile(filename.c_str());
        std::string line;
        int count = 0;

        while (std::getline(countfile, line)){
            if(line.find("facet normal") != std::string::npos){
                count++;
            }
        }
        countfile.close();
        //each points[i] contains one the three vertices of triangle i
        triangles.resize(count);
        normals.resize(count);
        //let's read the data
        std::ifstream infile(filename.c_str());
        std::getline(infile, line);
        int i =0;
        while (line.find("endsolid")==std::string::npos)
        {
            while (line.find("facet normal") == std::string::npos){std::getline(infile, line);}
            size_t pos = line.find("facet");
            line.erase(0, pos + 12);
            float aux;
            std::stringstream ss(line);
            ss >> std::skipws >>  aux; 
            normals[i].x = roundf(aux * roundFactor) / roundFactor;
            ss >> std::skipws >>  aux; 
            normals[i].y = roundf(aux * roundFactor) / roundFactor;
            ss >> std::skipws >>  aux; 
            normals[i].z = roundf(aux * roundFactor) / roundFactor;
            std::getline(infile, line);

            for(int j=0;j<3;j++){
                std::getline(infile, line);
                size_t pos = line.find("vertex ");
                line.erase(0, pos + 7);
                std::stringstream ss(line);
                ss >> std::skipws >>  aux; 
                triangles[i][j].x = roundf(aux * roundFactor) / roundFactor;
                ss >> std::skipws >>  aux; 
                triangles[i][j].y = roundf(aux * roundFactor) / roundFactor;
                ss >> std::skipws >>  aux; 
                triangles[i][j].z = roundf(aux * roundFactor) / roundFactor;
            }
            i++;
            //skipping lines here makes checking for the end of the file more convenient
            std::getline(infile, line);
            std::getline(infile, line);
            while(std::getline(infile, line)&&line.length()==0);
        }
        infile.close();
    }
    else{
        std::ifstream infile(filename.c_str(), std::ios_base::binary);
        infile.seekg(80 * sizeof(uint8_t), std::ios_base::cur); //skip the header
        uint32_t num_triangles;
        infile.read((char*) &num_triangles, sizeof(uint32_t));
        triangles.resize(num_triangles);
        normals.resize(num_triangles);

        for(int i = 0; i < num_triangles; i++){
            infile.read((char*) &normals[i], 3 * sizeof(float)); //read the normal vector
            
            for(int j=0; j<3;j++){
                infile.read((char*) &triangles[i][j], 3 * sizeof(float)); //read the point
            }

            infile.seekg(sizeof(uint16_t), std::ios_base::cur); //skip the attribute data
        }
        infile.close();
    }
    
    //OK, we have read the data, let's do something with it
    occupy(triangles, normals, value_to_write);

}

void findDimensions(std::string filename){
    bool ascii = false;
    if (FILE *file = fopen(filename.c_str(), "r"))
    {
        //File exists!, keep going!
        char buffer[6];
        fgets(buffer, 6, file);
        if(std::string(buffer).find("solid")!=std::string::npos)
            ascii=true;
        fclose(file);
    }else{
        std::cout<< "File " << filename << " does not exist\n";
        return;
    }


    if(ascii){
        //let's read the data
        std::string line;
        std::ifstream infile(filename.c_str());
        std::getline(infile, line);
        int i =0;
        while (line.find("endsolid")==std::string::npos)
        {
            while (std::getline(infile, line) && line.find("outer loop") == std::string::npos);

            for(int j=0;j<3;j++){
                float x, y, z;
                std::getline(infile, line);
                size_t pos = line.find("vertex ");
                line.erase(0, pos + 7);
                std::stringstream ss(line);
                float aux;
                ss >> std::skipws >>  aux; 
                x = roundf(aux * roundFactor) / roundFactor;
                ss >> std::skipws >>  aux; 
                y = roundf(aux * roundFactor) / roundFactor;
                ss >> std::skipws >>  aux; 
                z = roundf(aux * roundFactor) / roundFactor;
                env_max_x = env_max_x>=x?env_max_x:x;
                env_max_y = env_max_y>=y?env_max_y:y;
                env_max_z = env_max_z>=z?env_max_z:z;
                env_min_x = env_min_x<=x?env_min_x:x;
                env_min_y = env_min_y<=y?env_min_y:y;
                env_min_z = env_min_z<=z?env_min_z:z;
            }
            i++;
            //skipping three lines here makes checking for the end of the file more convenient
            std::getline(infile, line);
            std::getline(infile, line);
            while(std::getline(infile, line)&&line.length()==0);
        }
        infile.close();
    }
    else{
        std::ifstream infile(filename.c_str(), std::ios_base::binary);
        infile.seekg(80 * sizeof(uint8_t), std::ios_base::cur); //skip the header
        uint32_t num_triangles;
        infile.read((char*) &num_triangles, sizeof(uint32_t));
        for(int i = 0; i < num_triangles; i++){
            infile.seekg(3 * sizeof(float), std::ios_base::cur); //skip the normal vector
            for(int j=0; j<3;j++){
                float x, y ,z;
                infile.read((char*) &x, sizeof(float));
                infile.read((char*) &y, sizeof(float));
                infile.read((char*) &z, sizeof(float));
                env_max_x = env_max_x>=x?env_max_x:x;
                env_max_y = env_max_y>=y?env_max_y:y;
                env_max_z = env_max_z>=z?env_max_z:z;
                env_min_x = env_min_x<=x?env_min_x:x;
                env_min_y = env_min_y<=y?env_min_y:y;
                env_min_z = env_min_z<=z?env_min_z:z;
            }
            
            infile.seekg(sizeof(uint16_t), std::ios_base::cur); //skip the attribute data
        }
    }
    
    std::cout<<"Dimensions are:\n"<<
    "x: ("<<env_min_x<<", "<<env_max_x<<")\n"<<
    "y: ("<<env_min_y<<", "<<env_max_y<<")\n"<<
    "z: ("<<env_min_z<<", "<<env_max_z<<")\n";


}

int indexFrom3D(int x, int y, int z){
	return x + y*env[0].size() + z*env[0].size()*env.size();
}

void openFoam_to_gaden(std::string filename)
{

	//let's parse the file
	std::ifstream infile(filename.c_str());
	std::string line;

	//ignore the first line (column names)
	std::getline(infile, line);
    std::vector<double> U(env[0].size()*env.size()*env[0][0].size());
    std::vector<double> V(env[0].size()*env.size()*env[0][0].size());
    std::vector<double> W(env[0].size()*env.size()*env[0][0].size());
    std::vector<double> v(6);
	int x_idx = 0;
	int y_idx = 0;
	int z_idx = 0;
	while (std::getline(infile, line))
	{
		if (line.length()!=0)
		{
			for (int i = 0; i < 6; i++)
			{
				size_t pos = line.find(",");
				v[i] = atof(line.substr(0, pos).c_str());
				line.erase(0, pos + 1);
			}
			//assign each of the points we have information about to the nearest cell
			x_idx = (int)roundf((v[3] - env_min_x) / cell_size*roundFactor)/roundFactor;
			y_idx = (int)roundf((v[4] - env_min_y) / cell_size*roundFactor)/roundFactor;
			z_idx = (int)roundf((v[5] - env_min_z) / cell_size*roundFactor)/roundFactor;
			U[ indexFrom3D(x_idx, y_idx,z_idx) ] = v[0];
			V[ indexFrom3D(x_idx, y_idx,z_idx) ] = v[1];
			W[ indexFrom3D(x_idx, y_idx,z_idx) ] = v[2];
		}
	}
    infile.close();
    printWind(U,V,W,filename);
}

void fill(int x, int y, int z, cell_state new_value, cell_state value_to_overwrite){
    std::queue<Eigen::Vector3i> q;
    q.push(Eigen::Vector3i(x, y, z));
    env[x][y][z]=new_value;
    while(!q.empty()){
        Eigen::Vector3i point = q.front();
        q.pop();
        if(compare_cell(point.x()+1, point.y(), point.z(), value_to_overwrite)){ // x+1, y, z
            env[point.x()+1][point.y()][point.z()]=new_value;
            q.push(Eigen::Vector3i(point.x()+1, point.y(), point.z()));
        }

        if(compare_cell(point.x()-1, point.y(), point.z(), value_to_overwrite)){ // x-1, y, z
            env[point.x()-1][point.y()][point.z()]=new_value;
            q.push(Eigen::Vector3i(point.x()-1, point.y(), point.z()));
        }

        if(compare_cell(point.x(), point.y()+1, point.z(), value_to_overwrite)){ // x, y+1, z
            env[point.x()][point.y()+1][point.z()]=new_value;
            q.push(Eigen::Vector3i(point.x(), point.y()+1, point.z()));
        }

        if(compare_cell(point.x(), point.y()-1, point.z(), value_to_overwrite)){ // x, y-1, z
            env[point.x()][point.y()-1][point.z()]=new_value;
            q.push(Eigen::Vector3i(point.x(), point.y()-1, point.z()));
        }

        if(compare_cell(point.x(), point.y(), point.z()+1, value_to_overwrite)){ // x, y, z+1
            env[point.x()][point.y()][point.z()+1]=new_value;
            q.push(Eigen::Vector3i(point.x(), point.y(), point.z()+1));
        }

        if(compare_cell(point.x(), point.y(), point.z()-1, value_to_overwrite)){ // x, y, z-1
            env[point.x()][point.y()][point.z()-1]=new_value;
            q.push(Eigen::Vector3i(point.x(), point.y(), point.z()-1));
        }
    }
}

void clean(){
    for(int col=0;col<env.size();col++){
        for(int row=0;row<env[0].size();row++){
            for(int height=0;height<env[0][0].size();height++){
                
                if(env[col][row][height]==cell_state::edge){

                    if(compare_cell(col+1, row, height, cell_state::empty)||
                        compare_cell(col, row+1, height, cell_state::empty)||
                        compare_cell(col, row, height+1, cell_state::empty)||
                        (compare_cell(col+1, row+1, height, cell_state::empty)
                            &&env[col][row+1][height]==cell_state::edge
                            &&env[col+1][row][height]==cell_state::edge))
                    {
                        env[col][row][height]=cell_state::empty;
                    }else
                    {
                        env[col][row][height]=cell_state::occupied;
                    }
                    
                }
                
            }
        }
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "preprocessing");
    int numModels;
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("preprocessing_done",5,true);

    private_nh.param<float>("cell_size", cell_size, 1); //size of the cells

    roundFactor=100.0/cell_size;
    //stl file with the model of the outlets
    std::string outlet; int numOutletModels;

    //path to the csv file where we want to write the occupancy map
    std::string output;
    private_nh.param<std::string>("output_path", output, "");

    //--------------------------

        //OCCUPANCY

    //--------------------------

    private_nh.param<int>("number_of_models", numModels, 2); // number of CAD models
    
    std::vector<std::string> CADfiles;     
    for(int i = 0; i< numModels; i++){
        std::string paramName = boost::str( boost::format("model_%i") % i); //each of the stl models
        std::string filename;
        private_nh.param<std::string>(paramName, filename, "");
        CADfiles.push_back(filename.c_str());
    }

    for (int i = 0; i < CADfiles.size(); i++)
    {
        findDimensions(CADfiles[i]);
    }

    //x and y are interchanged!!!!!! it goes env[y][x][z]
    //I cannot for the life of me remember why I did that, but there must have been a reason
    env = std::vector<std::vector<std::vector<int> > > (ceil((env_max_y-env_min_y)*(roundFactor)/(cell_size*(roundFactor))),
                                                    std::vector<std::vector<int> >(ceil((env_max_x - env_min_x)*(roundFactor)/(cell_size*(roundFactor))),
                                                                                    std::vector<int>(ceil((env_max_z - env_min_z)*(roundFactor)/(cell_size*(roundFactor))), 0)));

    ros::Time start = ros::Time::now();
    for (int i = 0; i < numModels; i++)
    {
        parse(CADfiles[i], cell_state::occupied);
    }
      
    std::cout <<"Took "<< ros::Time::now().toSec()-start.toSec()<<" seconds \n";
    float empty_point_x;
    private_nh.param<float>("empty_point_x", empty_point_x, 1);
    float empty_point_y;
    private_nh.param<float>("empty_point_y", empty_point_y, 1);
    float empty_point_z;
    private_nh.param<float>("empty_point_z", empty_point_z, 1);

    //--------------------------

        //OUTLETS

    //--------------------------

    private_nh.param<int>("number_of_outlet_models", numOutletModels, 1); // number of CAD models

    std::vector<std::string> outletFiles;     
    for(int i = 0; i< numOutletModels; i++){
        std::string paramName = boost::str( boost::format("outlets_model_%i") % i); //each of the stl models
        std::string filename;
        private_nh.param<std::string>(paramName, filename, "");
        outletFiles.push_back(filename.c_str());
    }

    for (int i=0;i<numOutletModels; i++){
        parse(outletFiles[i], cell_state::outlet);
    }  

    std::cout<<"Filling...\n";
    //Mark all the empty cells reachable from the empty_point as aux_empty
    //the ones that cannot be reached will be marked as occupied when printing
    fill((empty_point_y-env_min_y)/cell_size,
        (empty_point_x-env_min_x)/cell_size,
        (empty_point_z-env_min_z)/cell_size, 
        cell_state::empty, cell_state::non_initialized);
    
    //get rid of the cells marked as "edge", since those are not truly occupied
    clean();

    private_nh.param<float>("floor_height", floor_height, 0); // number of CAD models
    printMap(boost::str(boost::format("%s/occupancy.pgm") % output.c_str()), 10);

    std::string worldFile;
    private_nh.param<std::string>("worldFile", worldFile, ""); // number of CAD models
    if(worldFile!="")
        changeWorldFile(worldFile);

    //output - path, occupancy vector, scale
    printEnv(boost::str(boost::format("%s/OccupancyGrid3D.csv") % output.c_str()), 1);
    printYaml(output);

    //-------------------------

        //WIND

    //-------------------------

    bool uniformWind;
    private_nh.param<bool>("uniformWind", uniformWind, false);

    //path to the point cloud files with the wind data
    std::string windFileName;
    private_nh.param<std::string>("wind_files", windFileName, "");
    int idx = 0;

    if(uniformWind){
        
        //let's parse the file
        std::ifstream infile(windFileName);
        std::string line;

        std::vector<double> U(env[0].size()*env.size()*env[0][0].size());
        std::vector<double> V(env[0].size()*env.size()*env[0][0].size());
        std::vector<double> W(env[0].size()*env.size()*env[0][0].size());
        while(std::getline(infile, line)){
            std::vector<double> v;
            for (int i = 0; i < 3; i++)
			{
				size_t pos = line.find(",");
				v.push_back(atof(line.substr(0, pos).c_str()));
				line.erase(0, pos + 1);
			}

            for(int i = 0; i< env[0].size();i++){
                for(int j = 0; j< env.size();j++){
                    for(int k = 0; k< env[0][0].size();k++){
                        if(env[j][i][k]==cell_state::empty){
                           
                            U[ indexFrom3D(i, j, k) ] = v[0];
                            V[ indexFrom3D(i, j, k) ] = v[1];
                            W[ indexFrom3D(i, j, k) ] = v[2];
                        }
                    }
                }
            }
            infile.close();
            printWind(U,V,W, boost::str(boost::format("%s_%i.csv") % windFileName % idx).c_str());
            idx++;
        }
    }else{
        while (FILE *file = fopen(boost::str(boost::format("%s_%i.csv") % windFileName % idx).c_str(), "r"))
        {
            fclose(file);
            openFoam_to_gaden(boost::str(boost::format("%s_%i.csv") % windFileName % idx).c_str());
            idx++;
        }
    }
    
    

    ROS_INFO("Preprocessing done");
    std_msgs::Bool b;
    b.data=true;
    pub.publish(b);
    
}
