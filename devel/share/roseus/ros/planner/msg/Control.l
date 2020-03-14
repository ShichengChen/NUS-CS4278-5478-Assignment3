;; Auto-generated. Do not edit!


(when (boundp 'planner::Control)
  (if (not (find-package "PLANNER"))
    (make-package "PLANNER"))
  (shadow 'Control (find-package "PLANNER")))
(unless (find-package "PLANNER::CONTROL")
  (make-package "PLANNER::CONTROL"))

(in-package "ROS")
;;//! \htmlinclude Control.msg.html


(defclass planner::Control
  :super ros::object
  :slots (_v _w ))

(defmethod planner::Control
  (:init
   (&key
    ((:v __v) 0.0)
    ((:w __w) 0.0)
    )
   (send-super :init)
   (setq _v (float __v))
   (setq _w (float __w))
   self)
  (:v
   (&optional __v)
   (if __v (setq _v __v)) _v)
  (:w
   (&optional __w)
   (if __w (setq _w __w)) _w)
  (:serialization-length
   ()
   (+
    ;; float64 _v
    8
    ;; float64 _w
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _v
       (sys::poke _v (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _w
       (sys::poke _w (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _v
     (setq _v (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _w
     (setq _w (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get planner::Control :md5sum-) "e63966e769215a305e1aebe679281346")
(setf (get planner::Control :datatype-) "planner/Control")
(setf (get planner::Control :definition-)
      "float64 v
float64 w
")



(provide :planner/Control "e63966e769215a305e1aebe679281346")


