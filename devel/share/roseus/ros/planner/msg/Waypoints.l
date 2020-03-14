;; Auto-generated. Do not edit!


(when (boundp 'planner::Waypoints)
  (if (not (find-package "PLANNER"))
    (make-package "PLANNER"))
  (shadow 'Waypoints (find-package "PLANNER")))
(unless (find-package "PLANNER::WAYPOINTS")
  (make-package "PLANNER::WAYPOINTS"))

(in-package "ROS")
;;//! \htmlinclude Waypoints.msg.html


(defclass planner::Waypoints
  :super ros::object
  :slots (_points ))

(defmethod planner::Waypoints
  (:init
   (&key
    ((:points __points) (let (r) (dotimes (i 0) (push (instance planner::Point :init) r)) r))
    )
   (send-super :init)
   (setq _points __points)
   self)
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:serialization-length
   ()
   (+
    ;; planner/Point[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; planner/Point[] _points
     (write-long (length _points) s)
     (dolist (elem _points)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; planner/Point[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance planner::Point :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get planner::Waypoints :md5sum-) "8f02263beef99aa03117a577a3eb879d")
(setf (get planner::Waypoints :datatype-) "planner/Waypoints")
(setf (get planner::Waypoints :definition-)
      "Point[] points
================================================================================
MSG: planner/Point
float64 x
float64 y
")



(provide :planner/Waypoints "8f02263beef99aa03117a577a3eb879d")


