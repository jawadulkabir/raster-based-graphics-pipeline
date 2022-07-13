#include<bits/stdc++.h>
 using namespace std;


ofstream stage1;

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
            stage1<<"vec "<<x<<" "<<y<<" "<<z<<endl;
        }
    };
    stack<double**>S;

public:
    void initialize()
    {
        cin>>eye_x>>eye_y>>eye_z>>look_x>>look_y>>look_z>>up_x>>up_y>>up_z>>fovY>>aspectRatio>>near>>far;
        modelingTransform();
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

    void modelingTransform()
    {
        string command;
        double** matrix = getIdentityMatrix();
        S.push(matrix);
        
        while(true)
        {
            cin>>command;
            if(command=="end") break;
            else if(command=="triangle")
            {
                struct vector p1,p2,p3;
                cin>>p1.x>>p1.y>>p1.z>>p2.x>>p2.y>>p2.z>>p3.x>>p3.y>>p3.z;

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

                
                double** temp = S.top();
                double** transformedTriangle = multiplyMatrix(temp,triangleMatrix);
                printTriangle(transformedTriangle);
            }
            else if(command=="translate")
            {
                struct vector t;
                cin>>t.x>>t.y>>t.z;

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
                cin>>angle>>r.x>>r.y>>r.z;
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
                cin>>s.x>>s.y>>s.z;

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

    void printTriangle(double** matrix)
    {
        for(int j=0;j<3;j++)
        {
            double x = matrix[0][j];
            double y = matrix[1][j];
            double z = matrix[2][j];
            stage1<<fixed<<std::setprecision(7)<<x<<" "<<y<<" "<<z<<endl;
        }
        stage1<<endl;
    }
};

int main()
{
    freopen("scene.txt","r",stdin);
    stage1.open("stage1.txt");

    Pipeline p;
    p.initialize();

}
