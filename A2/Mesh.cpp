#include "Mesh.h"

using namespace std;

void Mesh::load( const char* filename )
{
	// 2.1.1. load() should populate bindVertices, currentVertices, and faces

	// Add your code here.
    ifstream in;
    in.open(filename);
    
    if(in.is_open()) {
        string line;
        
        while(getline(in, line)) {
            Vector3f v;
            string s;
            Tuple3u f;
            stringstream ss;
            
            ss.str(line);
            
            ss >> s;
            
            if (s == "v") {
                ss >> v[0] >> v[1] >> v[2];
                bindVertices.push_back(v);
            }
            if (s == "f") {
                ss >> f[0] >> f[1] >> f[2];
                faces.push_back(f);
            }
            
        }
    }
    in.close();
    

	// make a copy of the bind vertices as the current vertices
	currentVertices = bindVertices;
}

void Mesh::draw()
{
	// Since these meshes don't have normals
	// be sure to generate a normal per triangle.
	// Notice that since we have per-triangle normals
	// rather than the analytical normals from
	// assignment 1, the appearance is "faceted".
    
    unsigned a, b, c;
    
    glBegin(GL_TRIANGLES);
    for (int i = 0; i < faces.size(); i++) {
        a = faces[i][0] - 1;
        b = faces[i][1] - 1;
        c = faces[i][2] - 1;
        
        Vector3f veca = currentVertices[b] - currentVertices[a];
        Vector3f vecb = currentVertices[c] - currentVertices[a];
        Vector3f norm = Vector3f::cross(veca, vecb);
        
        glNormal3d(norm[0], norm[1], norm[2]);
        glVertex3d(currentVertices[a][0], currentVertices[a][1], currentVertices[a][2]);
        glVertex3d(currentVertices[b][0], currentVertices[b][1], currentVertices[b][2]);
        glVertex3d(currentVertices[c][0], currentVertices[c][1], currentVertices[c][2]);
    }
    glEnd();
}

void Mesh::loadAttachments( const char* filename, int numJoints )
{
	// 2.2. Implement this method to load the per-vertex attachment weights
	// this method should update m_mesh.attachments
    ifstream in;
    in.open(filename);
    
    if(in.is_open()) {
        string line;
        
        while(getline(in, line)) {
            vector<float> attach;
            float value;
            stringstream ss;
            
            ss.str(line);
            
            while(ss) {
                ss >> value;
                attach.push_back(value);
            }
            
            attachments.push_back(attach);
        }
    }
    in.close();
}
