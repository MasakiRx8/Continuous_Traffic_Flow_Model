 #ファイル検索:リストアップ ターケットファイルリストの生成
 file(GLOB Target_Files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} CONFIGURE_DEPENDS *.cpp)

 #実行ファイルの生成
 add_executable(main ${Target_Files})