
// This is a test mex function that is based on a modified version of 
// BULLET's "helloWorld" example:
// http://bulletphysics.org/mediawiki-1.5.8/index.php/Hello_World

#include "mex.h"
#include "btBulletCollisionCommon.h"

void runBullet();

void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
    double *INPUT, *OUTPUT;
    
    plhs[0] = mxCreateDoubleMatrix(1,1, mxREAL);
    
    INPUT = mxGetPr(prhs[0]); 
    OUTPUT = mxGetPr(plhs[0]);
    
    OUTPUT[0] = 2.0 * INPUT[0];

    /// Collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    
    /////////////// IMPORTANT! //////////////////////////////////////
    collisionConfiguration->setConvexConvexMultipointIterations(); // Enables multiple contacts to be returned   
    /////////////////////////////////////////////////////////////////

    /// Use the default collision dispatcher. For parallel processing you can use a different dispatcher (see Extras/BulletMultiThreaded)
    btCollisionDispatcher* dispatcher = new	btCollisionDispatcher(collisionConfiguration);

    /// btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
    btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();

    // Keep track of the shapes, we release memory at exit.
    // Make sure to re-use collision shapes among rigid bodies whenever possible!
    btAlignedObjectArray<btCollisionShape*> collisionShapes;
   
    // A tetrahedral shape
    static btScalar VERTS[] = {
        -2.5, -2.5, -2.5,      // Cube
        -2.5, -2.5,  2.5,
        -2.5,  2.5, -2.5,
        -2.5,  2.5,  2.5,
         2.5, -2.5, -2.5,
         2.5, -2.5,  2.5,
         2.5,  2.5, -2.5,
         2.5,  2.5,  2.5,
        
        //0.,0.,3.6124, -0.2887, -.5, 2.7959, -.2887, 0.5, 2.7959, 0.5774, 0.0, 2.7959,   // World vert coords
       
       0.0, 0.0, 0.6124,       // Tetrahedron
      -0.2887, -0.5000, -0.2041,
      -0.2887,  0.5000, -0.2041,
       0.5774, 0, -0.2041
    };


    // Create collisionWorld
    btCollisionWorld *collisionWorld = new btCollisionWorld(dispatcher, overlappingPairCache,collisionConfiguration); 


    ////////////
    // A tetrahedron
    for (int t=1; t<=2; t++) {
        btConvexHullShape *shape = new btConvexHullShape; 
        if (t==1) {
            //shape = new btConvexHullShape(VERTS,8,3*sizeof(btScalar));      // Cube
            for (int v=0; v<8; v++) {
                shape->addPoint( btVector3( VERTS[3*v+0], VERTS[3*v+1], VERTS[3*v+2] ) );
            }
        }
        else {
            //shape = new btConvexHullShape(&VERTS[12],4,3*sizeof(btScalar)); // Tetrahedron
            for (int v=8; v<12; v++) {
                shape->addPoint( btVector3( VERTS[3*v+0], VERTS[3*v+1], VERTS[3*v+2] ) );
            }
        }
        collisionShapes.push_back(shape);
        
        
        // DEBUG
//         printf("Type: %d\n",shape.isConvex());
//         printf("Body 1 shape:\n");
//         int nv = shape.getNumVertices(); 
//         btVector3 *V;
//         for (int i=0; i<nv; i++) {
//             shape.getVertex(i, V); 
//             printf(" v%d: %f, %f, %f\n", i,V.x(),V.y(),V.z());
//         }
        // END DEBUG
        
         
        btCollisionObject *object1 = new btCollisionObject();
        object1->setCollisionShape(shape); 

        btTransform T1;
        T1.setIdentity();
        if (t==1)
            T1.setOrigin( btVector3(0., 0., 0.) );
        else
            T1.setOrigin( btVector3(0., 0., 2.5) );
        object1->setWorldTransform(T1);
        object1->setCompanionId(t);
        collisionWorld->addCollisionObject(object1);  // Add collision objects to collisionWorld
    }
   
    // Detect collision!!!
    collisionWorld->performDiscreteCollisionDetection(); 

    // Print out contacts
    int numManifolds = collisionWorld->getDispatcher()->getNumManifolds(); 

    if (num