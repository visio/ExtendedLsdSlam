
(cl:in-package :asdf)

(defsystem "myTestViewer-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "keyframeMsg" :depends-on ("_package_keyframeMsg"))
    (:file "_package_keyframeMsg" :depends-on ("_package"))
    (:file "keyframeGraphMsg" :depends-on ("_package_keyframeGraphMsg"))
    (:file "_package_keyframeGraphMsg" :depends-on ("_package"))
  ))