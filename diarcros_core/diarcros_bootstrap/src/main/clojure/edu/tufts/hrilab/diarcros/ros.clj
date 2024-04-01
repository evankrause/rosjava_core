(ns edu.tufts.hrilab.diarcros.ros
  ^{:doc "Convenience code for communication with the ROS master."
    :author "Jeremiah Via <jeremiah.via@gmail.com"}
  (:import org.ros.node.NodeConfiguration
           org.ros.internal.node.client.MasterClient
           org.ros.master.client.TopicType
           org.ros.namespace.GraphName
           org.ros.internal.node.server.ParameterServer
           org.ros.internal.node.service.ServiceTypeClient
           org.ros.address.InetAddressFactory
           java.net.URI)
  (:require [taoensso.timbre :as timbre
                      :refer (trace debug info warn error fatal spy)]))

;; For debugging
(timbre/set-level! :info)


(defn master-client
  "Wrapper function for creating MasterClient objects. Allows for
  communication to the XML-RPC API."
  ([]
     (def host (.toString (.getHostAddress (InetAddressFactory/newNonLoopback))))
     (def ros_master_uri (URI. (System/getenv "ROS_MASTER_URI")))
     (master-client (.getMasterUri (NodeConfiguration/newPublic host ros_master_uri))))
  ([uri]
     (trace "Creating master client.")
     (MasterClient. uri)))

(defn get-topic-types
  "Query the ROS master for a mapping between published topics and
  their types."
  [client]
  (let [response (.getResult (.getTopicTypes client (GraphName/root)))]
    (zipmap (map #(.getName %) response)
            (map #(.getMessageType %) response))))

(defn get-published-topics
  "Query the ROS master for a map of current topics and the nodes which
  publish and subsribe to them."
  [client]
  (let [response (.getTopics (.getResult (.getSystemState client (GraphName/root))))]
    (zipmap (map #(.getTopicName %) response)
            (map #(assoc {}
                    :publishers  (.getPublishers  %)
                    :subscribers (.getSubscribers %)) response))))

(defn get-service-type
  "Query the running ROS service to get the service type."
  [service-name client]
  (ServiceTypeClient/getServiceType service-name client))

(defn get-services
  "Query the ROS master for a map of current services and the nodes which
  provide them."
  [client]
  (let [response (.getServices (.getResult (.getSystemState client (GraphName/root))))]
    (zipmap (map #(.getServiceName %) response)
            (map #(.getServiceProviders %) response))))

(defn ros-nodes
  "Determine the nodes in the current system."
  [client]
  (let [topics (get-published-topics client)]
    (set (concat (mapcat :subscribers (vals topics))
                 (mapcat :publishers (vals topics))))))

(defn subscribes? [node topic client]
  "Given the name of a node (e.g., /stageros), determine if it
  subscribes to the given topic."
  (let [topics (get-published-topics client)]
    (contains? (:subscribers (get topics topic)) node)))

(defn subscriptions [node client]
  (info "Determining subscriptions for" node)
  (let [topics (get-topic-types client)]
    (into {}
          (filter (complement nil?)
                  (for [topic topics]
                    (when (subscribes? node (key topic) client)
                      {(key topic) (val topic)}))))))

(defn publishes? [node topic client]
  "Given the name of a node (e.g., /stageros), determine if it
  publishers to the given topic."
  (let [topics (get-published-topics client)]
    (contains? (:publishers (get topics topic)) node)))

(defn publications [node client]
  (info "Determining publications for" node)
  (let [topics (get-topic-types client)]
    (into {}
          (filter (complement nil?)
                  (for [topic topics]
                    (when (publishes? node (key topic) client)
                      {(key topic) (val topic)}))))))

(defn services [node client]
  (info "Determining services for" node)
  (let [services (get-services client)]
    (into {}
          (filter (complement nil?)
                  (for [service services]
                    (when (contains? (val service) node)
                      {(key service) (get-service-type (key service) client)}))))))


(defn ros-system-state
  "Determine the state of a running ROS system.

   This lists each node along with the topics to which is publishes
   and subscribes. The topics include the message type information"
  []
  (debug "Querying ROS system state.")
  (let [client (master-client)
        ;;topic-types (get-topic-types client)
        ;;topics (get-published-topics client)
        nodes (ros-nodes client)]
    (trace "Nodes seen:" nodes)
    (let [system-state (into {}
			     (for [node nodes]
				  (let [n {node {:subscriptions (subscriptions node client)
				                 :publications  (publications  node client)
                                 :services (services node client)}}]
                                    (debug n) n)))]
      (debug "System state" system-state)
      system-state)))

(defn rosnode-info [node]
  (debug "Querying node information.")
  (let [client (master-client)
        nodes  (ros-nodes client)]
    (if (contains? nodes node)
	  {:node node
	   :subscriptions (subscriptions node client)
	   :publications  (publications  node client)
	   :services  (services node client)}
          (fatal "No matching node in system"))))
