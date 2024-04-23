(ns edu.tufts.hrilab.diarcros.core
  ^{:doc "Code for generating ROS/DIARC bridge classes."
    :author "Jeremiah Via <jeremiah.via@gmail.com>"}
  (:use [clostache.parser :only [render-resource]]
        [clojure.tools.cli :only [cli]]
        [edu.tufts.hrilab.diarcros.ros :only [rosnode-info]])
  (:require [clojure.string :as str]
            [clojure.java.io :as io]
            [taoensso.timbre :as timbre
                      :refer (trace debug info warn error fatal spy)])
  (:gen-class :main true))


;; For debugging
(timbre/set-level! :info)


(def ros-class-exceptions
  "A mapping of ROS message types that do not follow the regular
  rosjava naming convention. These are used to handle the exception
  cases when creating the class." 
;;{"tf/tfMessage" "tf.tfMessage"}
  {"tf/tfMessage" "tf2_msgs.TFMessage"}
)

(defn strip-namespace [name]
  (str/replace name #"(/\w+)*/" ""))

(defn implements-action?
  "Determine if a node implements the actionlib interface.

   We're making the assumption that this is the case for any node
   which communicates through these topics: goal, cancel, feedback,
   status, and result. This is looks under namespaced topics."
  [node]
  (trace "Checking if node implements the action interface")
  (if (contains? node :actions) true
      (let [topics
            ;; We use this function for two different data structures, so
            ;; we need to get either one into the same representation.
            (if (contains? node :nodename)
              (set (concat
                    (map #(strip-namespace (:topic %)) (:subscriptions node))
                    (map #(strip-namespace (:topic %)) (:publications node))))
              (set (concat
                    (map strip-namespace (keys (:subscriptions node)))
		    (map strip-namespace (keys (:publications  node))))))]
	(trace "Looking at these topics:" topics)
        (and (contains? topics "goal")
             (contains? topics "cancel")
             (contains? topics "feedback")
             (contains? topics "status")
             (contains? topics "result")))))

(defn action-topic? [name]
  (let [topic (strip-namespace (key name))]
    (and (or (= "goal"     topic)
             (= "cancel"   topic)
             (= "feedback" topic)
             (= "status"   topic)
             (= "result"   topic))
         (not (.contains (key name) "simple")))))

(defn java-package
  "Convert a partial Java package path to its Java package equivalent."
  [path]
  (trace "Java-package-ize path:" path)
  (str/replace path #"/" "."))

(defn package-path
  "Convert a partial Java package specification to its path equivalent."
  [package]
  (trace "Java-path-ize package:" package)
  (str/replace package #"\." "/"))

(defn lower-case-first [s]
  [s]
  (trace "Convert first letter of string to lower case")
  (apply str (str/lower-case (first s)) (rest s)))

(defn upper-case-first [s]
  [s]
  (trace "Convert first letter of string to upper case" s)
  (if(nil? (first s))
    (str "")
    (apply str (str/upper-case (first s)) (rest s))))

(defn java-name
  "Convert a ROS topic/service name to a name which follows Java class naming
  conventions (e.g., /turtle_shape/get_loggers -> TurtleShapeGetLoggers)."
  [name]
  (trace "Java-ize name:" name)
  (let [components (str/split (str/replace name #"/" "_") #"_")]
    (apply str (map upper-case-first components))))
    ;;(apply str (map str/capitalize components))))

(defn java-type [typ]
  (trace "Determine Java type")
  (if (contains? ros-class-exceptions typ)
    (get ros-class-exceptions typ)
    (str/replace typ #"/" ".")))

(defn action-bases [act]
  (info "action-bases input" act)
  (let [aci (map (fn [a] [(strip-namespace (key a)) (val a)]) act)
       base (map (fn [a] [(if (= (first a) "feedback") (str/replace (second a) #"ActionFeedback" ""))]) aci)]
       (into [] (remove nil? (flatten base)))))
  ;;(let [base (map (fn [a] [(if (= (first a) "feedback") ((key a) (val str/replace (second a) #"ActionFeedback" "")))]) act)]
  ;;     (info "action-bases pre-nil" base)
  ;;     (into [] (remove nil? (flatten base)))))

(defn action-info [base]
  (info "action-info base" base)
  {:name (java-name base)
   :client-name base
   :class (str (java-type base) "Action.class")
   :action
   {:topic (str base "Action")
    :type  (java-type (str base "Action"))}
   :action-feedback
   {:topic (str base "ActionFeedback")
    :type  (java-type (str base "ActionFeedback"))}
   :action-goal
   {:topic (str base "ActionGoal")
    :type  (java-type (str base "ActionGoal"))}
   :action-result
   {:topic (str base "ActionResult")
    :type (java-type (str base "ActionResult"))}
   :feedback
   {:topic (str base "Feedback")
    :type  (java-type (str base "Feedback"))}
   :goal
   {:topic (str base "Goal")
    :type  (java-type (str base "Goal"))}
   :result
   {:topic (str base "Result")
    :type  (java-type (str base "Result"))}})

(defn ros->diarc [name package]
  (debug "ros->diarc")
  (let [node (rosnode-info name)
        info (fn [sub] (trace "info :: " sub)
		       {:name (java-name (key sub))
                :topic (key sub)
                :type (java-type (val sub))})
        srv-info (fn [sub] (trace "srv-info :: " sub)
		       {:name (java-name (key sub))
		        :rosname (key sub)
                :type (java-type (val sub))})]
    (trace "NODE" node)
    (if (implements-action? node)
      ;; Action node
      {:javanodename (java-name (:node node))
      :javapackage (java-package package)
      :people {{:name "Felix"} {:name "Jenny"}}
      :nodename (:node node)
      :publications (map info (filter (complement action-topic?)
                                      (:subscriptions node)))
      :subscriptions (map info (filter (complement action-topic?)
                                       (:publications node)))
      :services (map srv-info (:services node))
      :actions (map action-info (action-bases (filter action-topic?
                                (concat (:publications node)
                                        (:subscriptions node)))))}
      ;; Normal node
      {:javanodename (java-name (:node node))
       :javapackage (java-package package)
       :nodename (:node node)
       :publications (map info (:subscriptions node))
       :subscriptions (map info (:publications node))
       :services (map srv-info (:services node))})))


(def help-msg  "USAGE: ./gradlew diarcros -Dnodename=/node -Dpackage=package")

(defn -main
  "Entry point for the program."
  [& arg]
  (when (< (count arg) 2)
    (println help-msg)
    (System/exit 0))
  (println "Generating a DIARCROS node to communicate with" (nth arg 0))
  (try
    (when-let [node (ros->diarc (nth arg 0) (nth arg 1))]
      (trace "Node representation complete: " node)
      (def filepath (str "../../diarcros/" (package-path (:javapackage node)) "/" (:javanodename node) ".java"))
      (io/make-parents filepath)
      (if (implements-action? node)
        (spit (str filepath)
              (render-resource "action.mustache" node))
        (spit (str filepath)
	      (render-resource "node.mustache" node)))
      (println (str "\n  File edu/tufts/hrilab/diarcros/" (package-path (:javapackage node)) "/" (:javanodename node) ".java generated!")))
    
    (catch Exception e
      (println (.getCause e) "\n")
      (.printStackTrace e))))
