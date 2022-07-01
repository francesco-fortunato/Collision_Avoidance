;; Auto-generated. Do not edit!


(when (boundp 'collision_avoidance::custom)
  (if (not (find-package "COLLISION_AVOIDANCE"))
    (make-package "COLLISION_AVOIDANCE"))
  (shadow 'custom (find-package "COLLISION_AVOIDANCE")))
(unless (find-package "COLLISION_AVOIDANCE::CUSTOM")
  (make-package "COLLISION_AVOIDANCE::CUSTOM"))

(in-package "ROS")
;;//! \htmlinclude custom.msg.html


(defclass collision_avoidance::custom
  :super ros::object
  :slots (_started _linaer_velocity _angular_velocity ))

(defmethod collision_avoidance::custom
  (:init
   (&key
    ((:started __started) nil)
    ((:linaer_velocity __linaer_velocity) 0.0)
    ((:angular_velocity __angular_velocity) 0.0)
    )
   (send-super :init)
   (setq _started __started)
   (setq _linaer_velocity (float __linaer_velocity))
   (setq _angular_velocity (float __angular_velocity))
   self)
  (:started
   (&optional __started)
   (if __started (setq _started __started)) _started)
  (:linaer_velocity
   (&optional __linaer_velocity)
   (if __linaer_velocity (setq _linaer_velocity __linaer_velocity)) _linaer_velocity)
  (:angular_velocity
   (&optional __angular_velocity)
   (if __angular_velocity (setq _angular_velocity __angular_velocity)) _angular_velocity)
  (:serialization-length
   ()
   (+
    ;; bool _started
    1
    ;; float32 _linaer_velocity
    4
    ;; float32 _angular_velocity
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _started
       (if _started (write-byte -1 s) (write-byte 0 s))
     ;; float32 _linaer_velocity
       (sys::poke _linaer_velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angular_velocity
       (sys::poke _angular_velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _started
     (setq _started (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _linaer_velocity
     (setq _linaer_velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angular_velocity
     (setq _angular_velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get collision_avoidance::custom :md5sum-) "0c45fdadcf7f0ff78418b31fecc804fc")
(setf (get collision_avoidance::custom :datatype-) "collision_avoidance/custom")
(setf (get collision_avoidance::custom :definition-)
      "bool started
float32 linaer_velocity
float32 angular_velocity

")



(provide :collision_avoidance/custom "0c45fdadcf7f0ff78418b31fecc804fc")


