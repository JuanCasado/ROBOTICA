
function rosconnect ()
    URI = "http://172.29.30.28:11311";
    IP_LOCAL_MACHINE = "172.29.29.54";

    rosshutdown()
    rosinit(URI,"NodeHost",IP_LOCAL_MACHINE);
    pause(0.5)
end