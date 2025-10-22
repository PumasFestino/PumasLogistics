#!/bin/bash
# =========================================
# Script para instalar dependencias ROS 2 Jazzy
# =========================================

echo "Iniciando instalación de paquetes ROS 2 Jazzy..."

# Verificar si ROS 2 Jazzy está instalado
if ! printenv | grep -q "ROS_DISTRO=jazzy"; then
  echo "No se detectó ROS 2 Jazzy. Asegúrate de tenerlo instalado y haber hecho 'source /opt/ros/jazzy/setup.bash'"
  exit 1
fi

# Actualizar lista de paquetes
echo "Actualizando lista de paquetes..."
sudo apt update

# Instalar dependencias ROS y del sistema
echo "Instalando dependencias ROS 2 y de sistema..."
sudo apt install -y \
  ros-jazzy-control-msgs \
  ros-jazzy-urg-node \
  ros-jazzy-nav2-map-server \
  ros-jazzy-nav2-amcl \
  ros-jazzy-xacro \
  ros-jazzy-nav2-lifecycle-manager \
  espeak-ng

echo "Instalación completada correctamente."
