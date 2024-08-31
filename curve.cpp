#ifdef __APPLE__
#include <OpenGL/gl.h>
/* Just in case we need these later
// References:
// http://alumni.cs.ucsb.edu/~wombatty/tutorials/opengl_mac_osx.html
// # include <OpenGL/gl.h>
// # include <OpenGL/glu.h>
*/
#else
#include <GL/gl.h>
#endif

#include "curve.h"
#include "extra.h"
#ifdef WIN32
#include <windows.h>
#endif
using namespace std;

namespace
{
    // Approximately equal to.  We don't want to use == because of
    // precision issues with floating point.
    inline bool approx( const Vector3f& lhs, const Vector3f& rhs )
    {
        const float eps = 1e-8f;
        return ( lhs - rhs ).absSquared() < eps;
    }

    
}
    

Curve evalBezier( const vector< Vector3f >& P, unsigned steps )
{
    // Check
    if( P.size() < 4 || P.size() % 3 != 1 )
    {
        cerr << "evalBezier must be called with 3n+1 control points." << endl;
        exit( 0 );
    }

    // TODO:
    // You should implement this function so that it returns a Curve
    // (e.g., a vector< CurvePoint >).  The variable "steps" tells you
    // the number of points to generate on each piece of the spline.
    // At least, that's how the sample solution is implemented and how
    // the SWP files are written.  But you are free to interpret this
    // variable however you want, so long as you can control the
    // "resolution" of the discretized spline curve with it.

    // Make sure that this function computes all the appropriate
    // Vector3fs for each CurvePoint: V,T,N,B.
    // [NBT] should be unit and orthogonal.

    // Also note that you may assume that all Bezier curves that you
    // receive have G1 continuity.  Otherwise, the TNB will not be
    // be defined at points where this does not hold.

    cerr << "\t>>> evalBezier has been called with the following input:" << endl;

    cerr << "\t>>> Control points (type vector< Vector3f >): "<< endl;
    for( unsigned i = 0; i < P.size(); ++i )
    {
        cerr << "\t>>> " << P[i] << endl;
    }

    cerr << "\t>>> Steps (type steps): " << steps << endl;
    
    int seg;
    
    if (P.size() == 4) {
        seg = 1;
    }
    else {
        seg = ((P.size()-1)/3);
    }
    
    Curve R(seg * steps + 1);
    
    Vector3f BNormInit; //Initial arbitarty Bi-normal
    Vector3f BNormlast; //Most recent Bi-normal
    
    bool beginning = true;
    
    for(unsigned i = 0; i < P.size() - 3; i+=3) {
        if (beginning) {
            BNormInit = Vector3f(0.0f, 0.0f, 1.0f);
        } else {
            BNormInit = BNormlast;
        }
        
        for(unsigned d = 0; d <= steps; ++d) {
            
            float t = float(d) / steps;
            
            Matrix4f Points(P[i+0][0], P[i+1][0], P[i+2][0], P[i+3][0],
                              P[i+0][1], P[i+1][1], P[i+2][1], P[i+3][1],
                              P[i+0][2], P[i+1][2], P[i+2][2], P[i+3][2],
                              0.0f,0.0f,0.0f,0.0f);
            
            Matrix4f MatV(1.0f, -3.0f, 3.0f, -1.0f,
                          0.0f, 3.0f, -6.0f, 3.0f,
                          0.0f, 0.0f, 3.0f, -3.0f,
                          0.0f, 0.0f, 0.0f, 1.0f);
            
            Matrix4f MatT(-3.0f, 6.0f, -3.0f, 0.0f,
                          3.0f, -12.0f, 9.0f, 0.0f,
                          0.0f, 6.0f, -9.0f, 0.0f,
                          0.0f, 0.0f, 3.0f, 0.0f);
            
            Vector4f timeT(1, t, t*t, t*t*t);
            
            Vector4f VResult = Points * MatV * timeT;
            Vector4f TResult = Points * MatT * timeT;
            
            R[(i*steps) + d].V = Vector3f(VResult[0], VResult[1], VResult[2]);
            R[(i*steps) + d].T = Vector3f(TResult[0], TResult[1], TResult[2]).normalized();
            R[(i*steps) + d].N = Vector3f::cross(BNormInit, R[(i*steps) + d].T).normalized();
            R[(i*steps) + d].B = Vector3f::cross(R[(i*steps) + d].T, R[(i*steps) + d].N).normalized();
            
            
            
            BNormInit = R[(i*steps) + d].B;
            beginning = false;
            BNormlast = BNormInit;
        }
    }

    return R;
}

Curve evalBspline( const vector< Vector3f >& P, unsigned steps )
{
    // Check
    if( P.size() < 4 )
    {
        cerr << "evalBspline must be called with 4 or more control points." << endl;
        exit( 0 );
    }

    // TODO:
    // It is suggested that you implement this function by changing
    // basis from B-spline to Bezier.  That way, you can just call
    // your evalBezier function.

    cerr << "\t>>> evalBSpline has been called with the following input:" << endl;

    cerr << "\t>>> Control points (type vector< Vector3f >): "<< endl;
    for( unsigned i = 0; i < P.size(); ++i )
    {
        cerr << "\t>>> " << P[i] << endl;
    }

    cerr << "\t>>> Steps (type steps): " << steps << endl;
    
    int seg;
    
    if (P.size() == 4) {
        seg = 1;
    }
    else {
        seg = (P.size() - 3);
    }
    
    Curve R(seg * steps + 1);
    
    Vector3f BNormInit; //Initial arbitarty Bi-normal
    Vector3f BNormlast; //Most recent Bi-normal
    
    bool beginning = true;
    Vector3f storedN;
    Vector3f storedB;
    
    for(unsigned i = 0; i < P.size() - 3; ++i) {
        if (beginning) {
            BNormInit = Vector3f(0.0f, 0.0f, 1.0f);
        } else {
            BNormInit = BNormlast;
        }
        
        for(unsigned d = 0; d <= steps; ++d) {
            
            float t = float(d) / steps;
            
            Matrix4f Points(P[i+0][0], P[i+1][0], P[i+2][0], P[i+3][0],
                              P[i+0][1], P[i+1][1], P[i+2][1], P[i+3][1],
                              P[i+0][2], P[i+1][2], P[i+2][2], P[i+3][2],
                              0.0f,0.0f,0.0f,0.0f);
            
            Matrix4f MatV(1.0f/6, -3.0f/6, 3.0f/6, -1.0f/6,
                          4.0f/6, 0.0f/6, -6.0f/6, 3.0f/6,
                          1.0f/6, 3.0f/6, 3.0f/6, -3.0f/6,
                          0.0f, 0.0f, 0.0f, 1.0f/6);
            
            Matrix4f MatT(-3.0f/6,  6.0f/6, -3.0f/6, 0.0f/6,
                           0.0f/6, -12.0f/6, 9.0f/6, 0.0f/6,
                           3.0f/6,  6.0f/6, -9.0f/6, 0.0f/6,
                           0.0f/6,  0.0f/6,  3.0f/6, 0.0f/6);
            
            Vector4f timeT(1, t, t*t, t*t*t);
            
            Vector4f VResult = Points * MatV * timeT;
            Vector4f TResult = Points * MatT * timeT;
            
            R[(i*steps) + d].V = Vector3f(VResult[0], VResult[1], VResult[2]);
            R[(i*steps) + d].T = Vector3f(TResult[0], TResult[1], TResult[2]).normalized();
            
            R[(i*steps) + d].N = Vector3f::cross(BNormInit, 
                                                 R[(i*steps) + d].T).normalized();
            
            R[(i*steps) + d].B = Vector3f::cross(R[(i*steps) + d].T, 
                                                 R[(i*steps) + d].N).normalized();
            if (beginning) {
                storedN = R[(i*steps) + d].N;
                storedB = R[(i*steps) + d].B;
            }
            
            if (i + d == P.size() - 4 + steps) {
                R[(P.size() - 4)*steps+steps].N = storedN;
                R[(P.size() - 4)*steps+steps].B = storedB;
            }
            
            BNormInit = R[(i*steps) + d].B;
            
            beginning = false;
            BNormlast = BNormInit;
        }
    }
    
    
    
    cerr << "\t>>> Returning empty curve." << endl;
    
    return R;
}

Curve evalCircle( float radius, unsigned steps )
{
    // This is a sample function on how to properly initialize a Curve
    // (which is a vector< CurvePoint >).
    
    // Preallocate a curve with steps+1 CurvePoints
    Curve R( steps+1 );

    // Fill it in counterclockwise
    for( unsigned i = 0; i <= steps; ++i )
    {
        // step from 0 to 2pi
        float t = 2.0f * M_PI * float( i ) / steps;

        // Initialize position
        // We're pivoting counterclockwise around the y-axis
        R[i].V = radius * Vector3f( cos(t), sin(t), 0 );
        
        // Tangent vector is first derivative
        R[i].T = Vector3f( -sin(t), cos(t), 0 );
        
        // Normal vector is second derivative
        R[i].N = Vector3f( -cos(t), -sin(t), 0 );

        // Finally, binormal is facing up.
        R[i].B = Vector3f( 0, 0, 1 );
    }

    return R;
}

void drawCurve( const Curve& curve, float framesize )
{
    // Save current state of OpenGL
    glPushAttrib( GL_ALL_ATTRIB_BITS );

    // Setup for line drawing
    glDisable( GL_LIGHTING ); 
    glColor4f( 1, 1, 1, 1 );
    glLineWidth( 1 );
    
    // Draw curve
    glBegin( GL_LINE_STRIP );
    for( unsigned i = 0; i < curve.size(); ++i )
    {
        glVertex( curve[ i ].V );
    }
    glEnd();

    glLineWidth( 1 );

    // Draw coordinate frames if framesize nonzero
    if( framesize != 0.0f )
    {
        Matrix4f M;

        for( unsigned i = 0; i < curve.size(); ++i )
        {
            M.setCol( 0, Vector4f( curve[i].N, 0 ) );
            M.setCol( 1, Vector4f( curve[i].B, 0 ) );
            M.setCol( 2, Vector4f( curve[i].T, 0 ) );
            M.setCol( 3, Vector4f( curve[i].V, 1 ) );

            glPushMatrix();
            glMultMatrixf( M );
            glScaled( framesize, framesize, framesize );
            glBegin( GL_LINES );
            glColor3f( 1, 0, 0 ); glVertex3d( 0, 0, 0 ); glVertex3d( 1, 0, 0 );
            glColor3f( 0, 1, 0 ); glVertex3d( 0, 0, 0 ); glVertex3d( 0, 1, 0 );
            glColor3f( 0, 0, 1 ); glVertex3d( 0, 0, 0 ); glVertex3d( 0, 0, 1 );
            glEnd();
            glPopMatrix();
        }
    }
    
    // Pop state
    glPopAttrib();
}

