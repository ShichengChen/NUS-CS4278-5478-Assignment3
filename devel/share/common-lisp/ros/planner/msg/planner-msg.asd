
(cl:in-package :asdf)

(defsystem "planner-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Control" :depends-on ("_package_Control"))
    (:file "_package_Control" :depends-on ("_package"))
    (:file "ControlSequence" :depends-on ("_package_ControlSequence"))
    (:file "_package_ControlSequence" :depends-on ("_package"))
    (:file "Point" :depends-on ("_package_Point"))
    (:file "_package_Point" :depends-on ("_package"))
    (:file "Waypoints" :depends-on ("_package_Waypoints"))
    (:file "_package_Waypoints" :depends-on ("_package"))
  ))