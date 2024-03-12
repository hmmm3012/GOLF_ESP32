idf_component_register(SRCS "src/ws_echo_server.c"
                            "src/wifi.c"
                    INCLUDE_DIRS "."
                                "inc" 
                    EMBED_FILES "webpage/control.html")
