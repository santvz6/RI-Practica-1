#!/usr/bin/env python3
"""
=============================================================================
Práctica 1 - Robótica Inteligente (Grado en IA - EPS Universidad de Alicante)
Herramienta de Introspección Automática del Grafo ROS 2 para la Memoria
=============================================================================

Descripción:
Interroga la arquitectura dinámica de ROS 2 activa (Nodos, Tópicos, Tipos de
Mensajes, Servicios y Acciones) y exporta tablas estructuradas en Markdown
listas para ser incluidas en la Memoria Técnica de la práctica.
"""

import subprocess
import json
import sys


def run_ros_cmd(cmd_list):
    try:
        res = subprocess.run(cmd_list, capture_output=True, text=True, check=True)
        return res.stdout.strip().splitlines()
    except Exception as e:
        return []


def generate_introspection_report():
    print("# 📊 Reporte de Introspección del Middleware ROS 2\n")

    nodes = run_ros_cmd(['ros2', 'node', 'list'])
    print(f"## 1. Nodos Activos ({len(nodes)})\n")
    for n in nodes:
        print(f"- `{n}`")

    print("\n## 2. Mapeo Detallado de Nodos e Interfaces\n")
    print("| Nodo | Tópicos Suscritos (Inputs) | Tópicos Publicados (Outputs) | Servicios Ofrecidos |")
    print("| :--- | :--- | :--- | :--- |")

    for node in nodes:
        info_lines = run_ros_cmd(['ros2', 'node', 'info', node])
        subs, pubs, srvs = [], [], []
        curr_section = None
        for line in info_lines:
            line_str = line.strip()
            if 'Subscribers:' in line_str:
                curr_section = 'sub'
            elif 'Publishers:' in line_str:
                curr_section = 'pub'
            elif 'Service Servers:' in line_str:
                curr_section = 'srv'
            elif 'Action Servers:' in line_str or 'Action Clients:' in line_str:
                curr_section = 'act'
            elif line_str.startswith('/'):
                topic_name = line_str.split(':')[0]
                if curr_section == 'sub':
                    subs.append(topic_name)
                elif curr_section == 'pub':
                    pubs.append(topic_name)
                elif curr_section == 'srv':
                    srvs.append(topic_name)

        subs_str = "<br>".join(subs[:4]) + ("..." if len(subs) > 4 else "") if subs else "*(ninguno)*"
        pubs_str = "<br>".join(pubs[:4]) + ("..." if len(pubs) > 4 else "") if pubs else "*(ninguno)*"
        srvs_str = "<br>".join(srvs[:4]) + ("..." if len(srvs) > 4 else "") if srvs else "*(ninguno)*"

        print(f"| `{node}` | {subs_str} | {pubs_str} | {srvs_str} |")

    print("\n## 3. Acciones Activas\n")
    actions = run_ros_cmd(['ros2', 'action', 'list'])
    for a in actions:
        print(f"- `{a}`")


if __name__ == '__main__':
    generate_introspection_report()
