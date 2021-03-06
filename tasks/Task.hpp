/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef PANORAMICA_TASK_TASK_HPP
#define PANORAMICA_TASK_TASK_HPP

#include "panoramica/TaskBase.hpp"

namespace panoramica {

    /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Declare a new task context (i.e., a component)

The corresponding C++ class can be edited in tasks/Task.hpp and
tasks/Task.cpp, and will be put in the panoramica namespace.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','panoramica::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
     
     // Button names are in order in which they appear in the vector
    enum ButtonName
    {
        X,
        A,
        B,
        Y,
        LB,
        RB,
        LT,
        RT,
        BACK,
        START,
        LJOY,
        RJOY
    };
    
    // Pan-tilt axis names
    enum Axis
    {
        PAN = 0,
        TILT = 1
    };
     
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:
        static const double DEG2RAD = 3.14159/180;
        // For tilt angles they need to be multiplied by 4 to get them in proper units because of the gearing
        static const double TILT_MULTIPLIER = 4;
        
        // PTU pan and tilt angles from the PTU module (inputs for feedback)
        double pan_angle_in;
        double tilt_angle_in;
        // The tilt angle needs a multiplier, the value is stored in tilt_angle_temp
        // as tilt_angle_in in keeps getting overwritten for some reason...
        double tilt_angle_temp;
        
        // Position error margin for the pan and tilt positions
        double position_error_margin;
        int set_counter;
        
        // Vector containing the pan and tilt position for every picture
        std::vector<base::Vector2d> camera_positions;
        // Current position index
        unsigned int position_index;
        double pan_angle_goal;
        double tilt_angle_goal;
        
        base::Time goal_arrival_time;
        RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> left_frame, right_frame;
        
        base::Time frame_delay_um;
        bool save_frame;
        bool left_frame_saved;
        bool right_frame_saved;

        // Variables used for Tenerife field test where 360 panorama acquisitions are triggered externally with a fixed tilt angle
        double trigger_tilt;
        int sync;
        bool triggered;
        bool processed;
        bool processing;
        
    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "panoramica::Task");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
	~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
}

#endif

