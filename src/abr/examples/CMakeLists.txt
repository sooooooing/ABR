build_lib_example(
  NAME abr
  SOURCE_FILES abr.cc
  LIBRARIES_TO_LINK
    ${libwifi}
    ${libinternet}
    ${libabr}
    ${libinternet-apps}
)
