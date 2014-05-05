#include <iCub/iKin/iKinFwd.h>
#include <yarp/os/all.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Image.h>
#include <yarp/os/Stamp.h>
#include <iCub/stereoVision/sceneFlow.h>

using namespace std; 
using namespace yarp::os; 
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;

/**
* \ingroup 
*
* The base class defining the scene flow is in stereoVision.
* This adds more members in order to comply with doubleTouch.
*/

class sceneFlowThreadMod : public SceneFlow
{
private:

public:

    sceneFlowThreadMod(const yarp::os::ResourceFinder &rf);
    ~sceneFlowThreadMod(void) {};

    bool threadInit(void) {};
    void run(void) {}; 
    void close() {};
    void threadRelease() {};
};
