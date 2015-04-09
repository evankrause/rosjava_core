/*
 * Copyright (C) 2011 Alexander Perzylo, Technische Universität München
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ros.actionlib;

import org.ros.actionlib.client.ActionClient;
import org.ros.actionlib.client.SimpleActionClient;
import org.ros.actionlib.server.ActionServerCallbacks;
import org.ros.actionlib.server.DefaultActionServer;
import org.ros.actionlib.server.DefaultSimpleActionServer;
import org.ros.actionlib.server.SimpleActionServer;
import org.ros.actionlib.server.SimpleActionServerCallbacks;
import org.ros.exception.RosException;
import org.ros.internal.message.Message;
import org.ros.message.Time;
import org.ros.node.topic.*;
import actionlib_msgs.GoalID;
import actionlib_msgs.GoalStatus;
import std_msgs.Header;

import org.ros.node.NodeConfiguration;
import org.ros.message.MessageFactory;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
/**
 * 
 * An ActionSpec defines the action on which an action client and action server
 * communicate. It provides methods to create all necessary action messages and
 * extract specific pieces of data from given messages. ActionSpecs are needed
 * as a parameter to instantiate an action client or server. For the user's
 * convenience the ActionSpec class contains methods to build action clients and
 * servers. <br>
 * Example: <blockquote>
 * 
 * <pre>
 * {
 *   &#064;code
 *   ActionSpec&lt;FibonacciAction, FibonacciActionFeedback, FibonacciActionGoal, FibonacciActionResult, FibonacciFeedback, FibonacciGoal, FibonacciResult&gt; spec;
 * 
 *   spec =
 *       new ActionSpec&lt;FibonacciAction, FibonacciActionFeedback, FibonacciActionGoal, FibonacciActionResult, FibonacciFeedback, FibonacciGoal, FibonacciResult&gt;(
 *           FibonacciAction.class);
 * 
 *   SimpleActionClient&lt;FibonacciActionFeedback, FibonacciActionGoal, FibonacciActionResult, FibonacciFeedback, FibonacciGoal, FibonacciResult&gt; sac =
 *       spec.createSimpleActionClient(&quot;fibonacci&quot;);
 * }
 * </pre>
 * 
 * </blockquote> In order to cut the declaration part short, a specialized
 * ActionSpec can be derived and used (e.g. FibonacciActionSpec, which is part
 * of ROS package 'test_actionlib_java'): <blockquote>
 * 
 * <pre>
 * {
 *   &#064;code
 *   FibonacciActionSpec spec = new FibonacciActionSpec();
 *   FibonacciSimpleActionClient sac = spec.buildSimpleActionClient(&quot;fibonacci&quot;);
 * }
 * </pre>
 * 
 * </blockquote>
 * 
 * 
 * @author Alexander C. Perzylo, perzylo@cs.tum.edu
 * 
 * @param <T_ACTION>
 *          action message
 * @param <T_ACTION_FEEDBACK>
 *          action feedback message
 * @param <T_ACTION_GOAL>
 *          action goal message
 * @param <T_ACTION_RESULT>
 *          action result message
 * @param <T_FEEDBACK>
 *          feedback message
 * @param <T_GOAL>
 *          goal message
 * @param <T_RESULT>
 *          result message
 */
public class ActionSpec<
    T_ACTION extends Message, 
    T_ACTION_FEEDBACK extends Message, 
    T_ACTION_GOAL extends Message, 
    T_ACTION_RESULT extends Message, 
    T_FEEDBACK extends Message, 
    T_GOAL extends Message, 
    T_RESULT extends Message> {
  /**
   * Name of action
   */
  public final String actionName;

  /**
   * Class of action message
   */
  public final Class<T_ACTION> clsAction;

  /**
   * Class of action feedback message
   */
  public final Class<T_ACTION_FEEDBACK> clsActionFeedback;

  /**
   * Class of action goal message
   */
  public final Class<T_ACTION_GOAL> clsActionGoal;

  /**
   * Class of action result message
   */
  public final Class<T_ACTION_RESULT> clsActionResult;

  /**
   * Class of feedback message
   */
  public final Class<T_FEEDBACK> clsFeedback;

  /**
   * Class of goal message
   */
  public final Class<T_GOAL> clsGoal;

  /**
   * Class of result message
   */
  public final Class<T_RESULT> clsResult;
  
  /**
   * Class of Header message
   */

  public final Class<std_msgs.Header> clsHeader;

  private String actionMessage;
  private String actionFeedbackMessage;
  private String actionGoalMessage; 
  private String actionResultMessage;
  private String feedbackMessage;
  private String goalMessage;
  private String resultMessage;
  private NodeConfiguration nc;
  private MessageFactory mf;

  /**
   * Constructor. Checks if all needed fields are present in the given action
   * message class object and the referenced sub-messages. If there is something
   * missing, calling most of the methods of this class will result in a
   * NullPointerException. The isValid() method may be used to make sure the
   * ActionSpec was correctly instantiated.
   * 
   * @param clsAction
   *          The class object of an action message
   */
  @SuppressWarnings("unchecked")
  public ActionSpec(Class<T_ACTION> clsAction, 
      String actionMessage, 
      String actionFeedbackMessage, 
      String actionGoalMessage, 
      String actionResultMessage, 
      String feedbackMessage, 
      String goalMessage,
      String resultMessage) throws RosException {
      
    nc = NodeConfiguration.newPrivate();
    mf = nc.getTopicMessageFactory();

    this.actionMessage = actionMessage;
    this.actionFeedbackMessage = actionFeedbackMessage;
    this.actionGoalMessage = actionGoalMessage; 
    this.actionResultMessage = actionResultMessage;
    this.feedbackMessage = feedbackMessage;
    this.goalMessage = goalMessage;
    this.resultMessage = resultMessage;

    Class<T_ACTION> cA;
    Class<T_ACTION_FEEDBACK> cAF;
    Class<T_ACTION_GOAL> cAG;
    Class<T_ACTION_RESULT> cAR;
    Class<T_FEEDBACK> cF;
    Class<T_GOAL> cG;
    Class<T_RESULT> cR;
    /*
    Class<?> cA;
    Class<?> cAF;
    Class<?> cAG;
    Class<?> cAR;
    Class<?> cF;
    Class<?> cG;
    Class<?> cR;
    */
    String name;

    try {

      cA = clsAction;
      cAF = (Class<T_ACTION_FEEDBACK>) mf.newFromType(actionFeedbackMessage).getClass();
      cAG = (Class<T_ACTION_GOAL>) mf.newFromType(actionGoalMessage).getClass();
      cAR = (Class<T_ACTION_RESULT>) mf.newFromType(actionResultMessage).getClass();
      cF = (Class<T_FEEDBACK>) mf.newFromType(feedbackMessage).getClass();
      cG = (Class<T_GOAL>) mf.newFromType(goalMessage).getClass();
      cR = (Class<T_RESULT>) mf.newFromType(resultMessage).getClass();

      name = cA.getSimpleName();

    } catch (Exception e) {

      cA = null;
      cAF = null;
      cAG = null;
      cAR = null;
      cF = null;
      cG = null;
      cR = null;
      name = null;

      throw new RosException(
          "[ActionSpec] Wrong type definitions or action class used for ActionSpec instantiation!",
          e);
    }

    this.clsAction = clsAction;
    this.clsActionFeedback = cAF;
    this.clsActionGoal = cAG;
    this.clsActionResult = cAR;
    this.clsFeedback = cF;
    this.clsGoal = cG;
    this.clsResult = cR;
    this.actionName = name;
    this.clsHeader = std_msgs.Header.class;
  }

  /**
   * @return the actionMessage
   */
  public String getActionMessage() {
    return actionMessage;
  }

  /**
   * @return the actionFeedbackMessage
   */
  public String getActionFeedbackMessage() {
    return actionFeedbackMessage;
  }

  /**
   * @return the actionGoalMessage
   */
  public String getActionGoalMessage() {
    return actionGoalMessage;
  }

  /**
   * @return the actionResultMessage
   */
  public String getActionResultMessage() {
    return actionResultMessage;
  }

  /**
   * @return the feedbackMessage
   */
  public String getFeedbackMessage() {
    return feedbackMessage;
  }

  /**
   * @return the goalMessage
   */
  public String getGoalMessage() {
    return goalMessage;
  }

  /**
   * @return the resultMessage
   */
  public String getResultMessage() {
    return resultMessage;
  }

  /**
   * @return the actionName
   */
  public String getActionName() {
    return actionName;
  }

  /**
   * Checks, if entailed information is complete in order to use it with an
   * action client/server.
   * 
   * @return <tt>true</tt> - if this ActionSpec was instantiated correctly<br>
   *         <tt>false</tt> - otherwise (if that happens, please check if the
   *         given class parameters are correct and if their order complies with
   *         the class definition.)
   */
  public boolean isValid() {

    return (clsAction != null && clsActionFeedback != null && clsActionGoal != null
        && clsActionResult != null && clsFeedback != null && clsGoal != null && clsResult != null);

  }

  /**
   * Creates an ActionClient using this ActionSpec and a given nodeName space.
   * 
   * @param nameSpace
   *          The nodeName space to communicate within (specified by the action
   *          server)
   * @return An ActionClient object
   */
  public
      ActionClient<T_ACTION_FEEDBACK, T_ACTION_GOAL, T_ACTION_RESULT, T_FEEDBACK, T_GOAL, T_RESULT>
      buildActionClient(String nameSpace) {

    ActionClient<T_ACTION_FEEDBACK, T_ACTION_GOAL, T_ACTION_RESULT, T_FEEDBACK, T_GOAL, T_RESULT> ac =
        null;
    try {
      ac =
          new ActionClient<T_ACTION_FEEDBACK, T_ACTION_GOAL, T_ACTION_RESULT, T_FEEDBACK, T_GOAL, T_RESULT>(
              nameSpace, this);
    } catch (RosException e) {
      e.printStackTrace();
    }
    return ac;
  }

  /**
   * Creates a SimpleActionClient using this ActionSpec and a given nodeName space.
   * The SimpleActionClient gets parameterized to create a new thread to service
   * the callbacks. This spares the users the effort to call spin() or
   * spinOnce() themselves.
   * 
   * @param nameSpace
   *          The nodeName space to communicate within (specified by the action
   *          server)
   * @return A SimpleActionClient object
   */
  public
      SimpleActionClient<T_ACTION_FEEDBACK, T_ACTION_GOAL, T_ACTION_RESULT, T_FEEDBACK, T_GOAL, T_RESULT>
      buildSimpleActionClient(String nameSpace) {

    SimpleActionClient<T_ACTION_FEEDBACK, T_ACTION_GOAL, T_ACTION_RESULT, T_FEEDBACK, T_GOAL, T_RESULT> sac =
        null;
    try {
      sac =
          new SimpleActionClient<T_ACTION_FEEDBACK, T_ACTION_GOAL, T_ACTION_RESULT, T_FEEDBACK, T_GOAL, T_RESULT>(
              nameSpace, this);
    } catch (RosException e) {
      e.printStackTrace();
    }
    return sac;

  }

  /**
   * Creates an DefaultActionServer using this ActionSpec, a given nodeName space
   * and a callback object intended to be used by the server.
   * 
   * @param nameSpace
   *          The nodeName space to communicate within
   * @param callbacks
   *          A callback object providing callback methods, which get called by
   *          the server
   * @param autoStart
   *          A flag, indicating whether the server shall be immediately started
   *          or not
   * @return An DefaultActionServer object
   */
  public
      DefaultActionServer<T_ACTION_FEEDBACK, T_ACTION_GOAL, T_ACTION_RESULT, T_FEEDBACK, T_GOAL, T_RESULT>
      buildActionServer(
          String nameSpace,
          ActionServerCallbacks<T_ACTION_FEEDBACK, T_ACTION_GOAL, T_ACTION_RESULT, T_FEEDBACK, T_GOAL, T_RESULT> callbacks) {

    return new DefaultActionServer<T_ACTION_FEEDBACK, T_ACTION_GOAL, T_ACTION_RESULT, T_FEEDBACK, T_GOAL, T_RESULT>(
        nameSpace, this, callbacks);

  }

  /**
   * Creates a DefaultSimpleActionServer using this ActionSpec, a given nodeName
   * space and a callback object intended to be used by the server. The
   * useBlockingGoalCallback parameter specifies which callback method will be
   * used on the reception of goal messages.
   * 
   * @param nameSpace
   *          The nodeName space to communicate within
   * @param callbacks
   *          A callback object providing callback methods, which get called by
   *          the server
   * @param useBlockingGoalCallback
   *          A Flag, indicating whether the blocking or non-blocking callback
   *          method shall be used
   * @param autoStart
   *          A flag, indicating whether the server shall be immediately started
   *          or not
   * @return A DefaultSimpleActionServer object
   * 
   * @see SimpleActionServerCallbacks#blockingGoalCallback(Message)
   * @see SimpleActionServerCallbacks#goalCallback()
   */
  public
      SimpleActionServer<T_ACTION_FEEDBACK, T_ACTION_GOAL, T_ACTION_RESULT, T_FEEDBACK, T_GOAL, T_RESULT>
      buildSimpleActionServer(
          String nameSpace,
          SimpleActionServerCallbacks<T_ACTION_FEEDBACK, T_ACTION_GOAL, T_ACTION_RESULT, T_FEEDBACK, T_GOAL, T_RESULT> callbacks,
          boolean useBlockingGoalCallback) {

    return new DefaultSimpleActionServer<T_ACTION_FEEDBACK, T_ACTION_GOAL, T_ACTION_RESULT, T_FEEDBACK, T_GOAL, T_RESULT>(
        nameSpace, this, callbacks, useBlockingGoalCallback);

  }

  /**
   * Creates an action message.
   * 
   * @return A new action message object
   */
  public T_ACTION createActionMessage() {

    T_ACTION a = null;
    try {
      a = mf.newFromType(actionMessage);
    } catch (Exception e) {
    }
    return a;

  }

  /**
   * Creates an action feedback message.
   * 
   * @return A new action feedback message object
   */
  public T_ACTION_FEEDBACK createActionFeedbackMessage() {

    T_ACTION_FEEDBACK a = null;
    try {
      a = mf.newFromType(actionFeedbackMessage);
    } catch (Exception e) {
    }
    return a;

  }

  /**
   * Creates an action feedback message with the given feedback message,
   * timestamp and GoalStatus.
   * 
   * @param feedback
   *          A feedback message
   * @param t
   *          The timestamp of the action feedback message
   * @param gs
   *          The GoalStatus
   * @return A new action feedback message object
   */
  public T_ACTION_FEEDBACK createActionFeedbackMessage(T_FEEDBACK feedback, Publisher<T_ACTION_FEEDBACK> pubFeedback, Time t, GoalStatus gs) {
    T_ACTION_FEEDBACK a = null;
    try {
      a =  mf.newFromType(actionFeedbackMessage);
      std_msgs.Header header = mf.newFromType(std_msgs.Header._TYPE);
      String fm = feedbackMessage.replace('/','.');
      Method m = a.getClass().getMethod("setHeader", Header.class );
      header.setStamp(t);
      m.invoke(a, header);
      m = a.getClass().getMethod("setFeedback", Class.forName(fm));
      m.invoke(a, feedback);
      m = a.getClass().getMethod("setStatus", GoalStatus.class);
      m.invoke(a, gs);

    } catch (Exception e) {
      System.out.println("problem in AFM: " + e.toString());
      e.printStackTrace();
    }
    return a;

  }

  /**
   * Creates an action goal message.
   * 
   * @return A new action goal message object
   */
  public T_ACTION_GOAL createActionGoalMessage() {

    T_ACTION_GOAL a = null;
    try {
      a = mf.newFromType(actionGoalMessage);
    } catch (Exception e) {
    }
    return a;

  }

  /**
   * Creates an action goal message with the given goal message, timestamp and
   * GoalID.
   * 
   * @param goal
   *          A goal message
   * @param t
   *          The timestamp of the action goal message
   * @param goalID
   *          The goal's GoalID
   * @return A new action goal message object
   */
  public T_ACTION_GOAL createActionGoalMessage(T_GOAL goal, Time t, GoalID goalID) {
    T_ACTION_GOAL a = null;
    try {
      a =  mf.newFromType(actionGoalMessage);
      std_msgs.Header header = mf.newFromType(std_msgs.Header._TYPE);
      String gm = goalMessage.replace('/','.');
      Method m = a.getClass().getMethod("setHeader", Header.class );
      header.setStamp(t);
      m.invoke(a, header);
      m = a.getClass().getMethod("setGoal", Class.forName(gm));
      m.invoke(a, goal);
      m = a.getClass().getMethod("setGoalId", GoalID.class);
      m.invoke(a, goalID);

    } catch (Exception e) {
      System.out.println("problem in AGM: " + e.toString());
      e.printStackTrace();
    }
    return a;

  }

  /**
   * Creates an action result message.
   * 
   * @return A new action result message object
   */
  public T_ACTION_RESULT createActionResultMessage() {

    T_ACTION_RESULT a = null;
    try {
      a = mf.newFromType(actionResultMessage);
    } catch (Exception e) {
    }
    return a;

  }

  /**
   * Creates an action result message with the given result message, timestamp
   * and GoalStatus.
   * 
   * @param result
   *          A result message
   * @param t
   *          The timestamp of the action result message
   * @param gs
   *          The GoalStatus
   * @return A new action result message object
   */
  public T_ACTION_RESULT createActionResultMessage(T_RESULT result, Time t, GoalStatus gs) {
    T_ACTION_RESULT a = null;
    try {
      a =  mf.newFromType(actionResultMessage);
      std_msgs.Header header = mf.newFromType(std_msgs.Header._TYPE);
      String rm = resultMessage.replace('/','.');
      Method m = a.getClass().getMethod("setHeader", Header.class );
      header.setStamp(t);
      m.invoke(a, header);
      m = a.getClass().getDeclaredMethod("setResult", Class.forName(rm));
      m.invoke(a, result);
      m = a.getClass().getMethod("setStatus", GoalStatus.class);
      m.invoke(a, gs);

    } catch (Exception e) {
      System.out.println("problem in ARM: " + e.toString());
      e.printStackTrace();
    }
    return a;

  }

  /**
   * Creates a feedback message.
   * 
   * @return A new feedback message object
   */
  public T_FEEDBACK createFeedbackMessage() {

    T_FEEDBACK a = null;
    try {
      a = mf.newFromType(feedbackMessage);
    } catch (Exception e) {
    }
    return a;

  }

  /**
   * Creates a goal message.
   * 
   * @return A new goal message object
   */
  public T_GOAL createGoalMessage() {

    T_GOAL a = null;
    try {
      a = mf.newFromType(goalMessage);
    } catch (Exception e) {
    }
    return a;

  }

  /**
   * Creates a result message.
   * 
   * @return A new result message object
   */
  public T_RESULT createResultMessage() {

    T_RESULT a = null;
    try {
      a = mf.newFromType(resultMessage);
    } catch (Exception e) {
    }
    return a;

  }

  /**
   * Retrieves the feedback message from a given action feedback message.
   * 
   * @param actionFeedback
   *          An action feedback message
   * @return The contained feedback message
   */
  public T_FEEDBACK getFeedbackFromActionFeedback(T_ACTION_FEEDBACK actionFeedback)
      throws RosException {
    try {
      Method m = clsActionFeedback.getMethod("getFeedback");
      return clsFeedback.cast(m.invoke(actionFeedback));
    } catch (Exception e) {
      throw new RosException(
          "[ActionSpec] Couldn't find field 'feedback' in action feedback message.", e);
    }
  }

  /**
   * Retrieves the goal message from a given action goal message.
   * 
   * @param actionGoal
   *          An action goal message
   * @return The contained goal message
   */
  public T_GOAL getGoalFromActionGoal(T_ACTION_GOAL actionGoal) throws RosException {
    try {
      Method m = clsActionGoal.getMethod("getGoal");
      return clsGoal.cast(m.invoke(actionGoal));
    } catch (Exception e) {
      throw new RosException("[ActionSpec] Couldn't find field 'goal' in action goal message.", e);
    }
  }

  /**
   * Retrieves the result message from a given action result message.
   * 
   * @param actionResult
   *          An action result message
   * @return The contained result message
   */
  public T_RESULT getResultFromActionResult(T_ACTION_RESULT actionResult) throws RosException {
    try {
      Method m = clsActionResult.getMethod("getResult");
      return clsResult.cast(m.invoke(actionResult));
    } catch (Exception e) {
      throw new RosException("[ActionSpec] Couldn't find field 'result' in action result message.",
          e);
    }
  }

  /**
   * Retrieves the GoalID from a given action goal message.
   * 
   * @param actionGoal
   *          An action goal message
   * @return The contained GoalID
   */
  public GoalID getGoalIDFromActionGoal(T_ACTION_GOAL actionGoal) throws RosException {
    try {
      Method m = clsActionGoal.getMethod("getGoalId");
      return (GoalID) m.invoke(actionGoal);
    } catch (Exception e) {
      throw new RosException(
          "[ActionSpec] Couldn't find field 'goal_id' of type 'GoalID' in action goal message.", e);
    }
  }

  /**
   * Retrieves the GoalStatus from a given action feedback message.
   * 
   * @param actionFeedback
   *          An action feedback message
   * @return The contained GoalStatus
   */
  public GoalStatus getGoalStatusFromActionFeedback(T_ACTION_FEEDBACK actionFeedback)
      throws RosException {
    try {
      Method m = clsActionFeedback.getMethod("getStatus");
      return (GoalStatus) m.invoke(actionFeedback);
    } catch (Exception e) {
      throw new RosException(
          "[ActionSpec] Couldn't find field 'status' of type 'GoalStatus' in action feedback message.",
          e);
    }
  }

  /**
   * Retrieves the GoalStatus from a given action result message.
   * 
   * @param actionResult
   *          An action result message
   * @return The contained GoalStatus
   */
  public GoalStatus getGoalStatusFromActionResult(T_ACTION_RESULT actionResult) throws RosException {
    try {
      Method m = clsActionResult.getMethod("getStatus");
      return (GoalStatus) m.invoke(actionResult);
    } catch (Exception e) {
      throw new RosException(
          "[ActionSpec] Couldn't find field 'status' of type 'GoalStatus' in action result message.",
          e);
    }
  }

}
