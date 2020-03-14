;; Auto-generated. Do not edit!


(when (boundp 'planner::ControlSequence)
  (if (not (find-package "PLANNER"))
    (make-package "PLANNER"))
  (shadow 'ControlSequence (find-package "PLANNER")))
(unless (find-package "PLANNER::CONTROLSEQUENCE")
  (make-package "PLANNER::CONTROLSEQUENCE"))

(in-package "ROS")
;;//! \htmlinclude ControlSequence.msg.html


(defclass planner::ControlSequence
  :super ros::object
  :slots (_controls ))

(defmethod planner::ControlSequence
  (:init
   (&key
    ((:controls __controls) (let (r) (dotimes (i 0) (push (instance planner::Control :init) r)) r))
    )
   (send-super :init)
   (setq _controls __controls)
   self)
  (:controls
   (&rest __controls)
   (if (keywordp (car __controls))
       (send* _controls __controls)
     (progn
       (if __controls (setq _controls (car __controls)))
       _controls)))
  (:serialization-length
   ()
   (+
    ;; planner/Control[] _controls
    (apply #'+ (send-all _controls :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; planner/Control[] _controls
     (write-long (length _controls) s)
     (dolist (elem _controls)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; planner/Control[] _controls
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _controls (let (r) (dotimes (i n) (push (instance planner::Control :init) r)) r))
     (dolist (elem- _controls)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get planner::ControlSequence :md5sum-) "7b73a45fc9f216f397bef7a4afdff8b2")
(setf (get planner::ControlSequence :datatype-) "planner/ControlSequence")
(setf (get planner::ControlSequence :definition-)
      "Control[] controls
================================================================================
MSG: planner/Control
float64 v
float64 w
")



(provide :planner/ControlSequence "7b73a45fc9f216f397bef7a4afdff8b2")


