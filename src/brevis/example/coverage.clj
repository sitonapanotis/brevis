(ns brevis.example.coverage
  (:require [clojure.zip :as zip])
  (:use [brevis.graphics.basic-3D]
        [brevis.physics.space]
        [brevis.shape box sphere]
        [brevis.core]
        [cantor]))  

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Globals

(def num-robots 30)
(def avoidance (atom 0.2))
(def clustering (atom 0.1))
(def centering (atom 0.03))

(def max-acceleration 2)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Robots

(defn robot?
  "Is a thing a robot?"
  [thing]
  (= (:type thing) :robot))

(defn random-robot-position
  "Returns a random valid robot position."
  []
  (vec3 (- (rand 20) 10) 
        0
        (- (rand 20) 10)))

(defn random-robot-velocity
  "Returns a random reasonable velocity."
  []
  (vec3 (- (rand 2) 1) 0 (- (rand 2) 1)))

(defn make-robot
  "Make a new robot with the specified program. At the specified location."
  [position]
  (-> {}
      (make-real)
      (make-box)
;      (make-sphere)
      (assoc :type :robot
             :color [1 0 0]
             :position position)
      (make-collision-shape)))
  
(defn random-robot
  "Make a new random robot."
  []
  (make-robot (random-robot-position)))

(defn bound-acceleration
  "Keeps the acceleration within a reasonable range."
  [v]
  (if (> (length v) max-acceleration)
    (mul (div v (length v)) max-acceleration)
    v))

(defn fly
  "Change the acceleration of a robot."
  [robot dt nbrs]
  (let [closest-robot (first nbrs)
        centroid (div (reduce add (map :position nbrs)) 
                      (count nbrs))
        d-closest-robot (sub (:position closest-robot) (:position robot))
        d-centroid (sub centroid (:position robot))
        d-center (sub (vec3 0 0 0) (:position robot))]
    (assoc robot
           :acceleration (bound-acceleration
                           (add (:acceleration robot)
                                (mul d-center @centering)
                                (mul d-closest-robot @avoidance)
                                (mul d-centroid @clustering))))))  

(defn update-robot
  "Update a robot based upon its flocking behavior and the physical kinematics."
  [robot dt objects]  
  (let [objects (filter robot? objects)
        nbrs (sort-by-proximity (:position robot) objects)
        floor (some #(when (= (:type %) :floor) %) objects)]
    (doseq [el (:vertices (:shape robot))]
      (println el))
    (update-object-kinematics 
      (fly robot dt nbrs) dt)))

(add-update-handler :robot update-robot); This tells the simulator how to update these objects

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Collision handling
;;
;; Collision functions take [collider collidee] and return [collider collidee]
;;   Both can be modified; however, two independent collisions are actually computed [a b] and [b a].

(defn bump
  "Collision between two robots. This is called on [robot1 robot2] and [robot2 robot1] independently
so we only modify robot1."
  [robot1 robot2]
  [(assoc robot1 :color [(rand) (rand) (rand)])
   robot2])

(defn land
  "Collision between a robot and the floor."
  [robot floor]
  [(assoc robot
     :position (vec3 (.x (:position robot)) 0 (.z (:position robot)))
     :acceleration (vec3 0 0 0)
     :velocity (vec3 0 0 0))
   floor])

(add-collision-handler :robot :robot bump)
(add-collision-handler :robot :floor land)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; World updates

(defn initialize-simulation
  "This is the function where you add everything to the world."
  []
  (let [robots (repeatedly num-robots random-robot)
        floor (make-floor)]
    {:objects (conj robots floor)}))

(start-gui initialize-simulation update-world 0.1)
