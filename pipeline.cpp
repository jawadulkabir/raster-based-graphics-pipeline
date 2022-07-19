#include<bits/stdc++.h>
 using namespace std;

ifstream stage1in, stage2in, stage3in;
ofstream stage1out, stage2out, stage3out;

class Pipeline
{
    double eye_x,eye_y,eye_z;
    double look_x,look_y,look_z;
    double up_x,up_y,up_z;
    double fovY,aspectRatio,near,far;
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
    stack<double**>S;
    int triangleCount = 0;

public:
    void initialize()
    {
        stage1in>>eye_x>>eye_y>>eye_z>>look_x>>look_y>>look_z>>up_x>>up_y>>up_z>>fovY>>aspectRatio>>near>>far;
        modelingTransform();
        viewTransform();
        projectionTransform();
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

        // double projectionMatrix[4][4] = {
        //     {near/t,0,0,0},
        //     {0,near/r,0,0},
        //     {0,0,-(far+near)/(far-near),-2*far*near/(far-near)},
        //     {0,0,-1,0}
        // };
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
};

int main()
{
    
    stage1in.open("scene.txt"); 
    stage2in.open("stage1.txt"); 
    stage3in.open("stage2.txt"); 
    
    stage1out.open("stage1.txt");
    stage2out.open("stage2.txt");
    stage3out.open("stage3.txt");
    
    Pipeline p;
    p.initialize();

}
