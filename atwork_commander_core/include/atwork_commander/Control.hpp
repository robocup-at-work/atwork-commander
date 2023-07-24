#pragma once

#include <atwork_commander_msgs/RefboxState.h>
#include <atwork_commander_msgs/Task.h>

#include <boost/filesystem/path.hpp>

#include <string>
#include <vector>

namespace atwork_commander {

class ControlImpl;

/** Exceptions thrown by the Control interface of the refbox
 *
 * Each exception has a Reason and an optional argument.  The exceptions represents errors created through wrong
 * states, internal errors or wrong arguments passed.  The class inherites from std::runtime_error, which allows for
 * automatic printing of error messages in applications not catching the errors.
 **/
class ControlError : public std::runtime_error {
  public:

    /** Pre-defined error types **/
    enum class Reasons {
      PATH_INVALID,       ///< Filesystem path is invalid
      TASK_INVALID,       ///< Refbox does not contin a valid task
      STATE_INVALID,      ///< State of refbox does not fit command
      ARGUMENT_INVALID,   ///< Supplied argument is invalid for command
      NO_TASK,            ///< Task with the specified name does not exist
      NO_ROBOT,           ///< Refbox ha no robots registered
      SERVICE_ERROR,      ///< Internal error of the refbox
      CONNECTION_ERROR    ///< Connection to the refbox service could not be established
    };
  private:

    Reasons mReason;       ///< Type of error, see Reasons
    std::string mArgument; ///< Supplied argument of command
    std::string mCommand;  ///< Command that failed
  public:

    /** Constructor taking the error description
     *  
     *  \param command Command that created the error
     *  \param reason Type of error
     *  \param argument Optional argument describing the error
     **/
    ControlError(std::string command, Reasons reason, std::string argument="");

    /** Getter for type of error
     *
     *  \return Type of error
     **/
    Reasons reason() const { return mReason; }

    /** Getter of argument
     *
     *  \return optional argument of error, "" is unset
     **/
    const std::string& argument() const { return mArgument; }

    /** Getter of command
     *
     * \return command, which created the error
     **/
    const std::string& command() const { return mCommand; }
};

/** Interface class to the new RefBox of the \@Work League
 *
 * The interface class allows easy connection to the RefBox updating the RefboxState and supplying control commands to
 * change the state of the RefBox. The commands may throw a ControlError indicating errors in executing them. If no
 * exception is thrown the command is successfull.
 **/
class Control {
    ControlImpl* mImpl; ///< pImpl, Pointer to implementation to decouple interface and implementation
  public:
    using Robots = std::vector< std::string >;                           ///< Start argument type declaration
    using RefboxState = atwork_commander_msgs::RefboxState;              ///< State type of the RefBox
    using StateUpdateCallback = std::function<void(const RefboxState&)>; ///< Callback definition of state callback
    using Task = atwork_commander_msgs::Task;                            ///< Task type definition

    /** Constructor connecting to RefBox. **/
    Control();

    /** Destructur severing the connection to the RefBox. **/
    ~Control() noexcept;
    
    ///@{

    /** Setter of RefBox namespace
     *
     * \param name Namespace of RefBox to connect to
     **/
    void refbox(const std::string& name);

    /** Getter of RefBox namespace
     *
     * \return RefBox namespace
     **/
    const std::string& refbox() const;

    /** Getter of RefBox state. Should not be relyed on if accessed from another thread then the global callback queue
     * of ros is executed on.  Please use stateUpdateCallback to set a callback, which is executed in the same thread
     * context and enable manual thread synchronization.
     
     * \warning Thread-Safety: Unsafe
     
     * \return constant reference to RefBox state
     **/
     const RefboxState& state() const; void

     /** Setter of state update callback. State update callback can be a static function or lambda function or member
      * function. In case of member function std::bind is necessary to enable binding, e.g:
      * std::bind(&<MyClass>::<MyFunction>, this, std::placeholders::_1).
      *
      * \param callback to execute on update of RefboxState
      **/ stateUpdateCallback(StateUpdateCallback callback);

      /** Getter of state update callback
       *
       * \return currently set stateUpdateCallback
       **/
      const StateUpdateCallback& stateUpdateCallback() const;
      ///@}
      ///@{
      /** Command to generate a new Task on the RefBox
       
       * The old task will get replaced. Consider calling store to safe the current task.  Task will be randomly
       * generated using the configured task generator.
       
       * \dot Generate Command State Chart
       * digraph forward {
       *  node  [font=Helvetica, fontsize=10];
       *  rankdir = LR;
       *  IDLE -> READY [ font = Helvetica, fontsize=10, label="check(task) == true"];
       * }
       * \enddot
      
       * Allowed RefBox states: IDLE, READY
       * \see RefboxState
       
       * \throw ControlError
       
       * \param task the name of the task type to be generated
       **/
       Task generate(const std::string& task);

      /** Command to start the current Task of the RefBox. A list of robots to start the task on can be supplied. If
       * none is supplied the task will be started on all robots.  Supplied robot names need to be in form \<team
       * name\>/\<robot name\>.
       
       * \dot Start Command State Chart
       * digraph forward {
       *  node  [font=Helvetica, fontsize=10];
       *  rankdir = LR;
       *  READY -> PREPARATION;
       * }
       * \enddot
       
       * Allowed RefBox states: READY
       * \see RefboxState
       
       * \throw ControlError

       
       * \param robots List of robot names to start task on.
       **/
       void start( Robots robots = {} );

      /** Command to forward the current state of the RefBox manually. State forwarding is only allowed in PREPARATION
       * and EXECUTION phases according to:
       
       * \dot Forward Command State Chart
       * digraph forward {
       *  node  [font=Helvetica, fontsize=10];
       *  rankdir = LR;
       *  PREPARATION -> EXECUTION -> READY;
       * }
       * \enddot
       
       * Allowed RefBox states: PREPARATION, EXECUTION
       
       * \throw ControlError
       **/
      void forward();

      /** Command to stop the ongoing task of the RefBox
       
       * Stops the ongoing task by transition to READY from PREPARATION or EXECUTION
       
       * \dot Stop Command State Chart
       * digraph forward {
       *  node  [font=Helvetica, fontsize=10];
       *  rankdir = LR;
       *  IDLE -> IDLE;
       *  READY-> READY;
       *  PREPARATION -> READY;
       *  EXECUTION -> READY;
       * }
       * \enddot
       
       * Allowed RefBox states: IDLE, READY, PREPARATION, EXECUTION
       
       * \throw ControlError
       **/
      void stop();

      /** Command to store the current task of the RefBox locally. If no valid task is currently registered in the
       * RefBox the command will fail with a SERVICE_ERROR. If the supplied path is not existing it will be created. If
       * the supplied path is a directory or non-regular file, the command will fail with PATH_INVALID.
       
       * Allowed RefBox states: atwork_commander_msgs::RefboxState_#IDLE, atwork_commander_msgs::RefboxState_#READY, atwork_commander_msgs::RefboxState_#PREPARATION, atwork_commander_msgs::RefboxState_#EXECUTION
       
       * \throw ControlError
       * \param fileName filesystem path to store task to
       **/
       void store( boost::filesystem::path fileName );

       /** Command to load a locally stored task in the RefBox. The locally stored file will be parsed and submitted to
        * the RefBox, if no task is currently executed. If  the file does not exist or does not contain a valid task, a
        * PATH_INVALID ControlError will be thrown.
        
        * Allowed RefBox states: IDLE, READY
        
        * \throw ControlError
        * \param fileName filesystem path to load task from
        **/
        void load( boost::filesystem::path fileName );
        ///@}
    };
}
        
/** Overload of operator<< to enable printing of error types
 *
 * \param os An output stream, e.g. ROS_ERROR_STREAM or cout
 * \param r The specified error type
 *
 * \return Reference to supplied output stream for chaining
 **/
std::ostream& operator<<(std::ostream& os, atwork_commander::ControlError::Reasons r);
