#include<bits/stdc++.h>
#include <time.h>       /* time */

#include "bitmap_image.hpp"

 using namespace std;

ifstream stage1in, stage2in, stage3in, stage4in, configin;
ofstream stage1out, stage2out, stage3out, z_buffer_out;

class Pipeline
{
    double eye_x,eye_y,eye_z;
    double look_x,look_y,look_z;
    double up_x,up_y,up_z;
    double fovY,aspectRatio,near,far;

    double** z_buffer;

    int screen_height, screen_width;
    double x_left_limit, x_right_limit, y_bottom_limit, y_top_limit,z_front_limit,z_rear_limit; //pixel side coordinates

    double top_Y, left_X, right_X, bottom_Y; //pixel midpoint coordinates
    double dx,dy,z_max;

    struct vector
    {
        double x,y,z;
        vector()
        {
            x=0;
            y=0;
            z=0;
        }
        vector(double x,double y,double z)
        {
            this->x=x;
            this->y=y;
            this->z=z;
        }
        vector normalize()
        {
            double len=sqrt(x*x+y*y+z*z);
            return vector(x/len,y/len,z/len);
        }
        void print()
        {
            cout<<"vec "<<x<<" "<<y<<" "<<z<<endl;
        }

    };



    //Triangle struct is only used for stage 4 as instructed in spec
    struct Triangle
    {
        vector points[3];
        int color[3];
        double max_y;
        double min_y;
        
        void generateColor()
        {
            for(int i=0;i<3;i++)
            {
                rand();
                color[i]=rand()%255;
            }
        }
        

        Triangle()
        {
            points[0]=vector(0,0,0);
            points[1]=vector(0,0,0);
            points[2]=vector(0,0,0);
            max_y = 0;
            generateColor();;
        }

        Triangle(vector p1,vector p2,vector p3)
        {
            points[0]=p1;
            points[1]=p2;
            points[2]=p3;
            generateColor();
        }

        void sort()
        {
            std::sort(points, points+3, point_compare);
        }

        void compute_max_y(double topY)
        {
            max_y = max(max(points[0].y,points[1].y),points[2].y);
            max_y = min(max_y,topY);
        }

        void compute_min_y(double bottomY)
        {
            min_y = min(min(points[0].y,points[1].y),points[2].y);
            min_y = max(min_y,bottomY);
        }

        bool static point_compare(vector a, vector b)
        {
            return a.y>b.y;
        }

        void print()
        {
            cout<<"tri\n";
            points[0].print();
            points[1].print();
            points[2].print();
            cout<<"color "<<color[0]<<" "<<color[1]<<" "<<color[2]<<endl<<endl;
        }
    };
    
    stack<double**>S;
    int triangleCount = 0;

public:
    void initialize()
    {
        stage1in>>eye_x>>eye_y>>eye_z>>look_x>>look_y>>look_z>>up_x>>up_y>>up_z>>fovY>>aspectRatio>>near>>far;
        modelingTransform();
        viewTransform();
        projectionTransform();
        clipping_and_scan_conversion();

    }

    double get_intersecting_X(double x1, double y1, double x2, double y2, double ys)
    {
        return (x1+(x2-x1)*(ys-y1)/(y2-y1));
    }

    double get_intersecting_z(double y1, double z1, double y2, double z2, double ys)
    {
        return (z1+(z2-z1)*(ys-y1)/(y2-y1));
    }

    void clipping_and_scan_conversion()
    {
        configin>>screen_height>>screen_width>>x_left_limit>>y_bottom_limit>>z_front_limit>>z_rear_limit;
        x_right_limit = -x_left_limit;
        y_top_limit = -y_bottom_limit;

        dx = (x_right_limit-x_left_limit)/screen_width;
        dy = (y_top_limit-y_bottom_limit)/screen_height;

        top_Y = y_top_limit-dy/2.0; //representative line of the topmost pixel
        left_X = x_left_limit+dx/2.0; //representative line of the leftmost pixel
        right_X = x_right_limit-dx/2.0; //representative line of the rightmost pixel
        bottom_Y = y_bottom_limit+dy/2.0; //representative line of the bottommost pixel
        

        z_max = z_rear_limit;
        Triangle triangle[triangleCount];
        for(int i=0;i<triangleCount;i++)
        {
            stage4in>>triangle[i].points[0].x>>triangle[i].points[0].y>>triangle[i].points[0].z;
            stage4in>>triangle[i].points[1].x>>triangle[i].points[1].y>>triangle[i].points[1].z;
            stage4in>>triangle[i].points[2].x>>triangle[i].points[2].y>>triangle[i].points[2].z;
            triangle[i].sort();
        }


        //z buffer matrix initialization
        z_buffer = new double*[screen_height];
        for(int i=0;i<screen_height;i++)
        {
            z_buffer[i] = new double[screen_width];
            for(int j=0;j<screen_width;j++)
            {
                z_buffer[i][j] = z_max;
            }
        }

        //bitmap image initialization
        bitmap_image image(screen_width,screen_height);

        for(int k=0;k<triangleCount;k++)
        {   
            triangle[k].compute_max_y(top_Y);
            triangle[k].compute_min_y(bottom_Y);

            int row_start = round((top_Y - triangle[k].max_y)/dy);
            int row_end = round((top_Y - triangle[k].min_y)/dy);

            for(int i=row_start;i<=row_end;i++)
            {
                double ys = top_Y - i*dy;

                double y1, y2, y3; 
                double x1, x2, x3;
                double z2, z1, z3;
                y1 = triangle[k].points[0].y; x1 = triangle[k].points[0].x; z1 = triangle[k].points[0].z; 
                y2 = triangle[k].points[1].y; x2 = triangle[k].points[1].x; z2 = triangle[k].points[1].z; 
                y3 = triangle[k].points[2].y; x3 = triangle[k].points[2].x; z3 = triangle[k].points[2].z; 


                //calculate xa, xb, za, zb (intersecion points of the scanline)
                double left_intersecting_line, right_intersecting_line, za, zb;;
                if(ys > triangle[k].points[1].y || triangle[k].points[0].y == triangle[k].points[1].y)
                {
                    left_intersecting_line = get_intersecting_X(x1,y1,x2,y2,ys);
                    right_intersecting_line = get_intersecting_X(x1,y1,x3,y3,ys);
                    za = get_intersecting_z(y1,z1,y2,z2,ys);
                    zb = get_intersecting_z(y1,z1,y3,z3,ys);
                }           
                else
                {
                    left_intersecting_line = get_intersecting_X(x2,y2,x3,y3,ys);
                    right_intersecting_line = get_intersecting_X(x1,y1,x3,y3,ys);
                    za = get_intersecting_z(y2,z2,y3,z3,ys);
                    zb = get_intersecting_z(y1,z1,y3,z3,ys);
                }     

                if(left_intersecting_line>right_intersecting_line)
                {
                    swap(left_intersecting_line,right_intersecting_line);
                    swap(za,zb);
                }

                //min_y and max_y are triangle specific, but min_x and max_x are scanline specific
                double min_x = max(left_intersecting_line,left_X);  //min_x is xa
                double max_x = min(right_intersecting_line,right_X); //max_x is xb
                double xa = min_x;
                double xb = max_x;

                int left_column = round((min_x - left_X)/dx);
                int right_column = round((max_x - left_X)/dx);

                double row_const = (zb-za)/(xb-xa);

                for(int j=left_column;j<=right_column;j++)
                {
                    double zp;
                    double xp = left_X + j*dx;

                    if(j==left_column) zp = za + (xp-xa)*row_const;
                    else zp = zp + dx*row_const;


                    if(zp<z_buffer[i][j] && zp>=z_front_limit)
                    {
                        z_buffer[i][j] = zp;
                        double r = triangle[k].color[0];
                        double g = triangle[k].color[1];
                        double b = triangle[k].color[2];
                        image.set_pixel(j,i,r,g,b);
                    }
                }   
            }
        }

        image.save_image("out.bmp");

        for(int i =0;i<screen_height;i++)
        {
            for(int j=0;j<screen_width;j++)
            {
                if(z_buffer[i][j]<z_max)
                    z_buffer_out<<z_buffer[i][j]<<" ";
            }
                
            z_buffer_out<<endl;
        }
            

    }

    double** getIdentityMatrix()
    {
        double** matrix = new double*[4];
        for(int i=0;i<4;i++)
        {
            matrix[i] = new double[4];
            for(int j=0;j<4;j++)
            {
                if(i==j)
                    matrix[i][j] = 1.0;
                else
                    matrix[i][j] = 0;
            }
        }
        return matrix;
    }

    double** getTranslationMatrix(double x,double y,double z)
    {
        double** matrix = getIdentityMatrix();
        matrix[0][3] = x;
        matrix[1][3] = y;
        matrix[2][3] = z;
        return matrix;
    }

    double** getScalingMatrix(double x,double y,double z)
    {
        double** matrix = getIdentityMatrix();
        matrix[0][0] = x;
        matrix[1][1] = y;
        matrix[2][2] = z;
        return matrix;
    }

    double** getRotationMatrix(double theta,double x,double y,double z)
    {
        vector axis = vector(x,y,z).normalize();
        vector c1 = Rodrigues(axis,vector(1.0,0,0),theta);
        vector c2 = Rodrigues(axis,vector(0,1.0,0),theta);
        vector c3 = Rodrigues(axis,vector(0,0,1.0),theta);

        double** matrix = getIdentityMatrix();
        matrix[0][0] = c1.x;
        matrix[1][0] = c1.y;
        matrix[2][0] = c1.z;

        matrix[0][1] = c2.x;
        matrix[1][1] = c2.y;
        matrix[2][1] = c2.z;

        matrix[0][2] = c3.x;
        matrix[1][2] = c3.y;
        matrix[2][2] = c3.z;
        return matrix;
    }

    vector Rodrigues(vector a,vector X,double theta){
        vector Y = crossProduct(a,X);
        double d = dotProduct(a,X);
        double c = cos(theta);
        double s = sin(theta);
        double t = 1-c;
        return vector(
            c*X.x+s*Y.x+t*d*a.x,
            c*X.y+s*Y.y+t*d*a.y,
            c*X.z+s*Y.z+t*d*a.z
        );
    }

    vector crossProduct(vector a, vector b)
    {
        vector c = {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
        return c;
    }

    double dotProduct(vector a, vector b){
        return a.x*b.x+a.y*b.y+a.z*b.z;
    }

    double** getTriangleMatrix(vector p1, vector p2, vector p3)
    {
        double** triangleMatrix = getIdentityMatrix();
        triangleMatrix[0][0] = p1.x;
        triangleMatrix[1][0] = p1.y;
        triangleMatrix[2][0] = p1.z;
        triangleMatrix[3][0] = 1;
        
        triangleMatrix[0][1] = p2.x;
        triangleMatrix[1][1] = p2.y;
        triangleMatrix[2][1] = p2.z;
        triangleMatrix[3][1] = 1;
        
        triangleMatrix[0][2] = p3.x;
        triangleMatrix[1][2] = p3.y;
        triangleMatrix[2][2] = p3.z;
        triangleMatrix[3][2] = 1;

        return triangleMatrix;
    }

    void modelingTransform()
    {
        string command;
        double** matrix = getIdentityMatrix();
        S.push(matrix);
        
        while(true)
        {
            stage1in>>command;
            if(command=="end") break;
            else if(command=="triangle")
            {
                struct vector p1,p2,p3;
                stage1in>>p1.x>>p1.y>>p1.z>>p2.x>>p2.y>>p2.z>>p3.x>>p3.y>>p3.z;

                double** triangleMatrix = getTriangleMatrix(p1,p2,p3);

                
                double** temp = S.top();
                double** transformedTriangle = multiplyMatrix(temp,triangleMatrix);
                printTriangle(transformedTriangle,stage1out);
                triangleCount++;
            }
            else if(command=="translate")
            {
                struct vector t;
                stage1in>>t.x>>t.y>>t.z;

                double** matrix = getTranslationMatrix(t.x,t.y,t.z);
                double** top = S.top();
                double** newMatrix = multiplyMatrix(top,matrix);
                S.pop();
                S.push(newMatrix);
            }
            else if(command=="rotate")
            {
                struct vector r;
                double angle;
                stage1in>>angle>>r.x>>r.y>>r.z;
                double radian = angle*3.141592653589793/180;

                double** matrix = getRotationMatrix(radian,r.x,r.y,r.z);
                double** top = S.top();
                double** newMatrix = multiplyMatrix(top,matrix);
                S.pop();
                S.push(newMatrix);
            }
            else if(command=="scale")
            {
                struct vector s;
                stage1in>>s.x>>s.y>>s.z;

                double** matrix = getScalingMatrix(s.x,s.y,s.z);
                double** top = S.top();
                double** newMatrix = multiplyMatrix(top,matrix);
                S.pop();
                S.push(newMatrix);
            }
            else if(command=="push")
            {
                double** top = S.top();
                S.push(top);
            }
            else if(command=="pop")
            {
                S.pop();
            }
            
        }
    }

    void viewTransform()
    {

        vector eye(eye_x,eye_y,eye_z);
        vector look(look_x,look_y,look_z);
        vector up(up_x,up_y,up_z); 

        vector l(look.x-eye.x,look.y-eye.y,look.z-eye.z);
        l = l.normalize();
        vector r = crossProduct(l,up);
        r = r.normalize();
        vector u = crossProduct(r,l);

        double** viewTranslationMatrix = getIdentityMatrix();
        viewTranslationMatrix[0][3] = -eye.x;
        viewTranslationMatrix[1][3] = -eye.y;
        viewTranslationMatrix[2][3] = -eye.z;


        double** viewRotationMatrix = getIdentityMatrix();
        viewRotationMatrix[0][0] = r.x;
        viewRotationMatrix[0][1] = r.y;
        viewRotationMatrix[0][2] = r.z;

        viewRotationMatrix[1][0] = u.x;
        viewRotationMatrix[1][1] = u.y;
        viewRotationMatrix[1][2] = u.z;

        viewRotationMatrix[2][0] = -l.x;
        viewRotationMatrix[2][1] = -l.y;
        viewRotationMatrix[2][2] = -l.z;


        double** viewMatrix = multiplyMatrix(viewTranslationMatrix,viewRotationMatrix);
        
        vector p1,p2,p3;
        for(int i=0;i<triangleCount;i++)
        {
            stage2in>>p1.x>>p1.y>>p1.z>>p2.x>>p2.y>>p2.z>>p3.x>>p3.y>>p3.z;
            double** triangleMatrix = getTriangleMatrix(p1,p2,p3);
            double** transformedTriangle = multiplyMatrix(viewMatrix,triangleMatrix);
            printTriangle(transformedTriangle,stage2out);
        }

    }    

    void projectionTransform()
    {
        double fovX = fovY*aspectRatio;
        double t = near*tan(fovY*3.141592653589793/360);
        double r = near*tan(fovX*3.141592653589793/360);

        double** projectionMatrix = getIdentityMatrix();
        projectionMatrix[0][0] = near/r;
        projectionMatrix[1][1] = near/t;
        projectionMatrix[2][2] = -(far+near)/(far-near);
        projectionMatrix[2][3] = -2*far*near/(far-near);
        projectionMatrix[3][2] = -1;
        projectionMatrix[3][3] = 0;
         
        vector p1,p2,p3;
        for(int i=0;i<triangleCount;i++)
        {
            stage3in>>p1.x>>p1.y>>p1.z>>p2.x>>p2.y>>p2.z>>p3.x>>p3.y>>p3.z;
            double** triangleMatrix = getTriangleMatrix(p1,p2,p3);
            double** transformedTriangle = multiplyMatrix(projectionMatrix,triangleMatrix);
            for(int j=0;j<3;j++)
            {
                if(transformedTriangle[3][j]!=0)
                {
                    for(int i=0;i<4;i++)
                    {
                        transformedTriangle[i][j]/=transformedTriangle[3][j];
                    }
                }
            }
            printTriangle(transformedTriangle,stage3out);
        }
    }

    double** multiplyMatrix(double** arr1,double** arr2)
    {
        double** matrix = new double*[4];
        for(int i=0;i<4;i++)
        {
            matrix[i] = new double[4];
            for(int j=0;j<4;j++)
            {
                matrix[i][j] = 0;
                for(int k=0;k<4;k++)
                {
                    matrix[i][j] += arr1[i][k]*arr2[k][j];
                }
            }
        }
        return matrix;
    }

    void printTriangle(double** matrix, ofstream& stage)
    {
        for(int j=0;j<3;j++)
        {
            double x = matrix[0][j];
            double y = matrix[1][j];
            double z = matrix[2][j];
            double w = matrix[3][j];
            stage<<fixed<<std::setprecision(7)<<x<<" "<<y<<" "<<z<<endl;
        }
        stage<<endl;
    }

    ~Pipeline()
    {
        for(int i=0;i<screen_height;i++)
        {
            delete[] z_buffer[i];
        }
        delete[] z_buffer;
    }
};

int main()
{    
    stage1in.open("scene.txt"); 
    stage2in.open("stage1.txt"); 
    stage3in.open("stage2.txt"); 
    stage4in.open("stage3.txt");
    configin.open("config.txt");
    
    stage1out.open("stage1.txt");
    stage2out.open("stage2.txt");
    stage3out.open("stage3.txt");
    z_buffer_out.open("z_buffer.txt");
    
    Pipeline p;
    p.initialize();
}
