
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
    
    runBullet(); 
}

void runBullet() {
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

    // Create a box collision shape
    //btCollisionShape *boxShape = new btBoxShape(btVector3(1.,1.,1.));  
    //btCollisionShape *sphereShape = new btBoxShape(btVector3(1.0,1.0,1.0)); 
    
    // A sphere shape
    btCollisionShape *sphereShape = new btSphereShape(btScalar(1.0)); 

    // A tetrahedral shape
    static btScalar tetraVerts[] = {
       0.0, 0.0, 0.0,
       0.5, 0.0, 0.0,
       0.0, 0.5, 0.0,
       0.0, 0.0, 0.5
    };
    btCollisionShape *tetraShape = new btConvexHullShape(tetraVerts,4,3*sizeof(btScalar)); 

    // A pyramid shape
    btConvexHullShape *pyramidShape = new btConvexHullShape();
    pyramidShape->addPoint( btVector3(-.5,-.5,0.0) );
    pyramidShape->addPoint( btVector3(-.5,0.5,0.0) );
    pyramidShape->addPoint( btVector3(0.5,-.5,0.0) );
    pyramidShape->addPoint( btVector3(0.5,0.5,0.0) );
    pyramidShape->addPoint( btVector3(0.0,0.0,0.5) );

    // A box shape
    btConvexHullShape *boxShape = new btConvexHullShape();
    boxShape->addPoint( btVector3(-1.,-1.,-1.) ); 
    boxShape->addPoint( btVector3(-1.,-1.,1.) ); 
    boxShape->addPoint( btVector3(-1.,1.,-1.) ); 
    boxShape->addPoint( btVector3(-1.,1.,1.) ); 
    boxShape->addPoint( btVector3(1.,-1.,-1.) ); 
    boxShape->addPoint( btVector3(1.,-1.,1.) ); 
    boxShape->addPoint( btVector3(1.,1.,-1.) ); 
    boxShape->addPoint( btVector3(1.,1.,1.) ); 

    printf("Created mesh with %d verts, %d edges, %d faces. \n",pyramidShape->getNumPoints(), pyramidShape->getNumEdges(), pyramidShape->getNumPlanes());
    for (int v=0; v<pyramidShape->getNumVertices(); v++) {
        btVector3 vert; 
        pyramidShape->getVertex(v,vert);
        printf("  [%d] %f, %f, %f \n", v, vert.x(), vert.y(), vert.z());
    }

    // Create collision objects 
    //////////////
    // A cube
    btCollisionObject *cubeObject = new btCollisionObject();
    cubeObject->setCollisionShape(boxShape);
    btTransform cubeTransform;
    cubeTransform.setIdentity();
    cubeTransform.setOrigin( btVector3(0.,0.,0.) );
            cubeTransform.setRotation(btQuaternion(btVector3(1.,0.,0.), 0.0));
    cubeObject->setWorldTransform(cubeTransform);
    cubeObject->setCompanionId(1);      // We use companionIds as body IDs

    /////////////
    // A sphere
    btCollisionObject *sphereObject = new btCollisionObject();
    sphereObject->setCollisionShape(sphereShape);
    btTransform sphereTransform;
    sphereTransform.setIdentity();
    sphereTransform.setOrigin( btVector3(0.,0.1,2.01) ); 
            sphereTransform.setRotation(btQuaternion(btVector3(1.,0.,0.), 3.14159));
    sphereObject->setWorldTransform(sphereTransform); 
    sphereObject->setCompanionId(2); 

    ////////////
    // A tetrahedron
    btCollisionObject *tetraObject = new btCollisionObject();
    tetraObject->setCollisionShape(tetraShape);
    btTransform tetraTransform;
    tetraTransform.setIdentity();
    tetraTransform.setOrigin( btVector3(0., 0., 1.0) );
    tetraObject->setWorldTransform(tetraTransform);
    tetraObject->setCompanionId(3);

    ////////////
    // A pyramid
    btCollisionObject *pyramidObject = new btCollisionObject();
    pyramidObject->setCollisionShape(pyramidShape);
    btTransform pyramidTransform;
    pyramidTransform.setIdentity();
    pyramidTransform.setOrigin( btVector3(0., 0., 1.0) );
    pyramidObject->setWorldTransform(pyramidTransform);
    pyramidObject->setCompanionId(4);


    // Add collision shapes to the shape vector
    collisionShapes.push_back(boxShape);
    collisionShapes.push_back(sphereShape); 
    collisionShapes.push_back(tetraShape);
    collisionShapes.push_back(pyramidShape); 

    // Create collisionWorld
    btCollisionWorld *collisionWorld = new btCollisionWorld(dispatcher, overlappingPairCache,collisionConfiguration); 

    // Add collision objects to collisionWorld
    collisionWorld->addCollisionObject(cubeObject);
    //collisionWorld->addCollisionObject(sphereObject); 
    collisionWorld->addCollisionObject(tetraObject); 
    //collisionWorld->addCollisionObject(pyramidObject); 

    // Detect collision!!!
    collisionWorld->performDiscreteCollisionDetection(); 

    // Print out contacts
    int numManifolds = collisionWorld->getDispatcher()->getNumManifolds(); 

    if (numManifolds == 0)
        printf("No contacts found!\n");
    else {
        printf("NumManifolds: %d\n", numManifolds);

        for (int i=0;i<numManifolds;i++)
        {
            btPersistentManifold* contactManifold = collisionWorld->getDispatcher()->getManifoldByIndexInternal(i);
            //btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0()); 
            //btCollisionObject* obB = contactManifold->getBody1();

            printf("  MANIFOLD: %d\n", i);
            int numContacts = contactManifold->getNumContacts();
            //printf("   Bodies: %d, %d\n", contactManifold->getBody0()->getCompanionId(), contactManifold->getBody1()->getCompanionId());
            for (int j=0;j<numContacts;j++)
            {
                printf("    CONTACT: %d, (%d, %d)\n", j, contactManifold->getBody0()->getCompanionId(), contactManifold->getBody1()->getCompanionId());
                btManifoldPoint& pt = contactManifold->getContactPoint(j);
                btVector3 ptA = pt.getPositionWorldOnA();
                btVector3 ptB = pt.getPositionWorldOnB();
                printf("      pA = (%f,%f,%f)\n",ptA.x(),ptA.y(),ptA.z());
                printf("      pB = (%f,%f,%f)\n",ptB.x(),ptB.y(),ptB.z());
                btVector3 n = pt.m_normalWorldOnB; 
                printf("      psi_n = %f,  Bnorm = (%f,%f,%f)\n",pt.getDistance(), n.x(), n.y(), n.z());

            }
        }
    }


    //cleanup in the reverse order of creation/initialization

    ///-----cleanup_start-----

    //delete collision shapes
    for (int j=0;j<collisionShapes.size();j++)
    {
        btCollisionShape* shape = collisionShapes[j];
        collisionShapes[j] = 0;
        delete shape;
    }

    delete collisionWorld; 

    //delete broadphase
    delete overlappingPairCache;

    //delete dispatcher
    delete dispatcher;

    delete collisionConfiguration;

    //next line is optional: it will be cleared by the destructor when the array goes out of scope
    collisionShapes.clear();

    ///-----cleanup_end-----   
}


