idf_build_get_property(project_dir PROJECT_DIR)

# Initialize an empty list to hold conditional requirements
set(conditionally_required_components "")

if(EXISTS "${project_dir}/components/drv_rmt")
    list(APPEND conditionally_required_components "drv_rmt")
endif()

idf_component_register(SRCS "drv_dht11.c" 
                    INCLUDE_DIRS "." 
                    REQUIRES
                        driver
                        ${conditionally_required_components} 
                                      )
                 

