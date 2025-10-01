cmake_minimum_required(VERSION 3.8)
project(oxebots_strategy)

# Configurações de compilador para avisos úteis
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encontra todas as dependências necessárias para o projeto
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(oxebots_interfaces REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Seção para configuração de testes
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# --- Definição do Executável ÚNICO ---

# Crie um único executável que inclui todos os seus arquivos .cpp de nós
add_executable(
  strategy_node
  src/strategy_node.cpp
  src/go_to_point_node.cpp
)

# Aplique todas as dependências necessárias ao executável principal
ament_target_dependencies(
  strategy_node
  rclcpp
  oxebots_interfaces
  behaviortree_cpp
  ament_index_cpp
)

# Adicione os diretórios de include locais do seu pacote
target_include_directories(strategy_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# --- Instalação ---

# Instale apenas o executável principal
install(TARGETS
  strategy_node
  DESTINATION lib/${PROJECT_NAME}
)

# Instala o diretório de includes
install(
  DIRECTORY include/
  DESTINATION include
)

# Instala arquivos de configuração, como a árvore de comportamento
install(
  FILES test_tree.xml
  DESTINATION share/${PROJECT_NAME}
)

# Instala o diretório de launch files
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

# Finaliza a configuração do pacote
ament_package()