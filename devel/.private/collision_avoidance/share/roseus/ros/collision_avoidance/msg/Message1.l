;; Auto-generated. Do not edit!


(when (boundp 'collision_avoidance::Message1)
  (if (not (find-package "COLLISION_AVOIDANCE"))
    (make-package "COLLISION_AVOIDANCE"))
  (shadow 'Message1 (find-package "COLLISION_AVOIDANCE")))
(unless (find-package "COLLISION_AVOIDANCE::MESSAGE1")
  (make-package "COLLISION_AVOIDANCE::MESSAGE1"))

(in-package "ROS")
;;//! \htmlinclude Message1.msg.html


(defclass collision_avoidance::Message1
  :super ros::object
  :slots (_partito _linaer_velocity _angular_velocity ))

(defmethod collision_avoidance::Message1
  (:init
   (&key
    ((:partito __partito) nil)
    ((:linaer_velocity __linaer_velocity) 0.0)
    ((:angular_velocity __angular_velocity) 0.0)
    )
   (send-super :init)
   (setq _partito __partito)
   (setq _linaer_velocity (float __linaer_velocity))
   (setq _angular_velocity (float __angular_velocity))
   self)
  (:partito
   (&optional __partito)
   (if __partito (setq _partito __partito)) _partito)
  (:linaer_velocity
   (&optional __linaer_velocity)
   (if __linaer_velocity (setq _linaer_velocity __linaer_velocity)) _linaer_velocity)
  (:angular_velocity
   (&optional __angular_velocity)
   (if __angular_velocity (setq _angular_velocity __angular_velocity)) _angular_velocity)
  (:serialization-length
   ()
   (+
    ;; bool _partito
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
     ;; bool _partito
       (if _partito (write-byte -1 s) (write-byte 0 s))
     ;; float32 _linaer_velocity
       (sys::poke _linaer_velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angular_velocity
       (sys::poke _angular_velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _partito
     (setq _partito (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _linaer_velocity
     (setq _linaer_velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angular_velocity
     (setq _angular_velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get collision_avoidance::Message1 :md5sum-) "a0f279fd92a14d96829a9ff564b4fe61")
(setf (get collision_avoidance::Message1 :datatype-) "collision_avoidance/Message1")
(setf (get collision_avoidance::Message1 :definition-)
      "bool partito
float32 linaer_velocity
float32 angular_velocity

")



(provide :collision_avoidance/Message1 "a0f279fd92a14d96829a9ff564b4fe61")


