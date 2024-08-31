#include "SkeletalModel.h"

#include <FL/Fl.H>

using namespace std;

void SkeletalModel::load(const char *skeletonFile, const char *meshFile, const char *attachmentsFile)
{
	loadSkeleton(skeletonFile);

	m_mesh.load(meshFile);
	m_mesh.loadAttachments(attachmentsFile, m_joints.size());

	computeBindWorldToJointTransforms();
	updateCurrentJointToWorldTransforms();
}

void SkeletalModel::draw(Matrix4f cameraMatrix, bool skeletonVisible)
{
	// draw() gets called whenever a redraw is required
	// (after an update() occurs, when the camera moves, the window is resized, etc)

	m_matrixStack.clear();
	m_matrixStack.push(cameraMatrix);

	if( skeletonVisible )
	{
		drawJoints();

		drawSkeleton();
	}
	else
	{
		// Clear out any weird matrix we may have been using for drawing the bones and revert to the camera matrix.
		glLoadMatrixf(m_matrixStack.top());

		// Tell the mesh to draw itself.
		m_mesh.draw();
	}
}

void SkeletalModel::loadSkeleton( const char* filename )
{
	// Load the skeleton from file here.
    ifstream in;
    in.open(filename);
    
    if(in.is_open()) {
        string line;
        Vector3f cor;
        int index;
        
        if(getline(in, line)) {
            stringstream ss;
            
            ss.str(line);
            ss >> cor[0] >> cor[1] >> cor[2] >> index;
            //create Joint
            Joint *joint = new Joint;
            
            
            joint->transform = Matrix4f::translation(cor);
            
            m_rootJoint = joint;
            m_joints.push_back(joint);
            
        }
        
        while(getline(in, line)) {
            stringstream ss;
            
            ss.str(line);
            ss >> cor[0] >> cor[1] >> cor[2] >> index;
            
            //create Joint
            Joint *joint = new Joint;
            joint->transform = Matrix4f::translation(cor);
            
            m_joints.push_back(joint);
            
            m_joints[index] -> children.push_back(joint);
            
        }
    }
    in.close();
}


void SkeletalModel::recur_joint(Joint *node) {
   
    m_matrixStack.push(node->transform);
    
    glLoadMatrixf(m_matrixStack.top());
    glutSolidSphere( 0.025f, 12, 12 );
    
    for (int i = 0; i < node->children.size(); ++i) {
        recur_joint(node->children[i]);
    }
    
    m_matrixStack.pop();
}

void SkeletalModel::drawJoints( )
{
	// Draw a sphere at each joint. You will need to add a recursive helper function to traverse the joint hierarchy.
	//
	// We recommend using glutSolidSphere( 0.025f, 12, 12 )
	// to draw a sphere of reasonable size.
	//
	// You are *not* permitted to use the OpenGL matrix stack commands
	// (glPushMatrix, glPopMatrix, glMultMatrix).
	// You should use your MatrixStack class
	// and use glLoadMatrix() before your drawing call.
    Matrix4f root = m_rootJoint->transform;
    m_matrixStack.push(root);
    glLoadMatrixf(m_matrixStack.top());
    for (int i = 0; i < m_rootJoint->children.size(); ++i) {
        recur_joint(m_rootJoint->children[i]);
    }
    
    m_matrixStack.pop();
    glLoadMatrixf(m_matrixStack.top());
}

void SkeletalModel::recur_bone(Joint *node) {
    if (!node->children.empty()) {
        m_matrixStack.push(node->transform);
        for (int i = 0; i < node->children.size(); ++i) {
            //Rotation
            Vector3f rnd( 0, 0, 1);
            Vector3f new_z = node->children[i]->transform.getCol(3).xyz();
            new_z.normalize();
            
            Vector3f y = Vector3f::cross(new_z, rnd).normalized();
            Vector3f x = Vector3f::cross(y, new_z).normalized();
            
            Matrix3f basis(x, y, new_z);
            
            Matrix4f b = Matrix4f::identity();
            b.setSubmatrix3x3(0,0, basis);
            
            // add transforms
            m_matrixStack.push(b);
            m_matrixStack.push(Matrix4f::scaling(0.025,0.025, node->children[i]->transform.getCol(3).xyz().abs()));
            m_matrixStack.push(Matrix4f::translation(0,0,0.5));
            
            // Draw Cube
            glLoadMatrixf(m_matrixStack.top());
            glutSolidCube(1);
            
            m_matrixStack.pop();
            m_matrixStack.pop();
            m_matrixStack.pop();
            recur_bone(node->children[i]);
        }
        m_matrixStack.pop();
    }
}

void SkeletalModel::drawSkeleton( )
{
	// Draw boxes between the joints. You will need to add a recursive helper function to traverse the joint hierarchy.
    recur_bone(m_rootJoint);
}

void SkeletalModel::setJointTransform(int jointIndex, float rX, float rY, float rZ)
{
	// Set the rotation part of the joint's transformation matrix based on the passed in Euler angles.
    
    Matrix4f curr = m_joints[jointIndex]->transform;
    curr.setSubmatrix3x3(0,0,(Matrix4f::rotateZ(rZ) * Matrix4f::rotateY(rY) * Matrix4f::rotateX(rX)).getSubmatrix3x3(0, 0));
    
    m_joints[jointIndex]->transform = curr;
}

void SkeletalModel::BindWorldtoJointHelper(Joint *j) {
//    Matrix4f temp = m * j->transform;
//    j->bindWorldToJointTransform = temp;
//    
//    for(int i = 0; i < j->children.size(); i++){
//        BindWorldtoJointHelper(j->children[i], temp);
//    }
    
    m_matrixStack.push(j->transform);
    j->bindWorldToJointTransform = m_matrixStack.top();
    
    for (int i = 0; i < j->children.size(); i++) {
        BindWorldtoJointHelper(j->children[i]);
    }
    m_matrixStack.pop();
}


void SkeletalModel::computeBindWorldToJointTransforms()
{
	// 2.3.1. Implement this method to compute a per-joint transform from
	// world-space to joint space in the BIND POSE.
	//
	// Note that this needs to be computed only once since there is only
	// a single bind pose.
	//
	// This method should update each joint's bindWorldToJointTransform.
	// You will need to add a recursive helper function to traverse the joint hierarchy.
//    BindWorldtoJointHelper(m_rootJoint, m_rootJoint->transform);
    
    m_matrixStack.clear();
    m_matrixStack.push(m_rootJoint->transform);
    m_rootJoint->bindWorldToJointTransform = m_rootJoint->transform;
    
    for (int i = 0; i < m_rootJoint->children.size(); i++) {
        BindWorldtoJointHelper(m_rootJoint->children[i]);
    }
    m_matrixStack.pop();
}

void SkeletalModel::CurrentJointToWorldHelper(Joint *j) {
//    Matrix4f temp = j->transform.inverse() * m;
//    j->currentJointToWorldTransform = temp;
//    
//    for(int i = 0; i < j->children.size(); i++){
//        CurrentJointToWorldHelper(j->children[i], temp);
//    }
    m_matrixStack.push(j->transform);
    j->currentJointToWorldTransform = m_matrixStack.top();
    
    for (int i = 0; i < j->children.size(); i++) {
        CurrentJointToWorldHelper(j->children[i]);
    }
    m_matrixStack.pop();
    
    
}

void SkeletalModel::updateCurrentJointToWorldTransforms()
{
	// 2.3.2. Implement this method to compute a per-joint transform from
	// joint space to world space in the CURRENT POSE.
	//
	// The current pose is defined by the rotations you've applied to the
	// joints and hence needs to be *updated* every time the joint angles change.
	//
	// This method should update each joint's bindWorldToJointTransform.
	// You will need to add a recursive helper function to traverse the joint hierarchy.
//    CurrentJointToWorldHelper(m_rootJoint, m_rootJoint->transform.inverse());
    
    m_matrixStack.clear();
    m_matrixStack.push(m_rootJoint->transform);
    m_rootJoint->currentJointToWorldTransform = m_rootJoint->transform;
    
    for (int i = 0; i < m_rootJoint->children.size(); i++) {
        CurrentJointToWorldHelper(m_rootJoint->children[i]);
    }
    m_matrixStack.pop();
}

void SkeletalModel::updateMesh()
{
	// 2.3.2. This is the core of SSD.
	// Implement this method to update the vertices of the mesh
	// given the current state of the skeleton.
	// You will need both the bind pose world --> joint transforms.
	// and the current joint --> world transforms.
    vector<Vector3f> newVertices;
    
    for (int i = 0; i < m_mesh.currentVertices.size(); i++) {
        Vector4f sumPoints(0.0f);
        Vector4f p(m_mesh.bindVertices[i], 1.0f);
        
        for (int j = 0; j < m_joints.size() - 1; ++j) {
            if(m_mesh.attachments[i][j] > 0) {
                sumPoints = sumPoints + m_mesh.attachments[i][j] * (m_joints[j+1] -> currentJointToWorldTransform * m_joints[j+1] -> bindWorldToJointTransform.inverse() * p);
            }
        }
        newVertices.push_back(sumPoints.xyz());
        
    }
    m_mesh.currentVertices = newVertices;
}

