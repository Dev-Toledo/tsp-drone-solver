# src/solver.py

import gurobipy as gp
from gurobipy import GRB
import math
import matplotlib.pyplot as plt
import os
import glob
import time

def distancia(i, j):
    """Calcula a distância euclidiana entre dois pontos (tuplas x,y)."""
    return math.sqrt((j[0] - i[0])**2 + (j[1] - i[1])**2)

def plot_rota(file_name, n, x, y, coordenadas, output_dir="images"):
    """
    Gera e salva o gráfico das rotas do caminhão e do drone.
    """
    # Cria diretório de imagens se não existir
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # 1) Identifica arcos percorridos
    arcos_caminhao = [[i, j] for i in range(n) for j in range(n) if i != j and x[i, j].x > 0.5]
    arcos_drone = [[i, j] for i in range(n) for j in range(n) if i != j and y[i, j].x > 0.5]

    # 2) Prepara coordenadas para plotagem
    # Nota: Em um cenário real, você iteraria sobre os arcos para desenhar.
    # Aqui simplificamos para plotar os nós e as conexões diretas.
    
    plt.figure(figsize=(12, 8))
    
    # Separa X e Y para scatter plot
    X_coords = [c[0] for c in coordenadas]
    Y_coords = [c[1] for c in coordenadas]
    
    # Plota Caminhão (Círculos Azuis) e Drone (X Vermelhos - apenas destinos)
    # Na prática, todos são nós, mas diferenciamos visualmente quem visita
    plt.scatter(X_coords, Y_coords, c='black', zorder=1)

    # Desenha rotas do caminhão (Linha Azul Sólida)
    for i, j in arcos_caminhao:
        p1, p2 = coordenadas[i], coordenadas[j]
        plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'b-', alpha=0.7, linewidth=2, label='Truck' if i==0 and j==arcos_caminhao[0][1] else "")

    # Desenha rotas do drone (Linha Vermelha Tracejada)
    for i, j in arcos_drone:
        p1, p2 = coordenadas[i], coordenadas[j]
        plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'r--', alpha=0.7, linewidth=1.5, label='Drone' if i==arcos_drone[0][0] else "")

    # Destaque para o Depósito
    plt.plot(coordenadas[0][0], coordenadas[0][1], 'gs', markersize=12, label='Depot')

    plt.title(f'Otimização de Rota: {file_name}')
    plt.legend(loc='best')
    
    # Salva o arquivo
    save_path = os.path.join(output_dir, f"{file_name}.png")
    plt.savefig(save_path, dpi=300)
    print(f"Gráfico salvo em: {save_path}")
    plt.close()

def solve_tsp_drone(R, K, coordenadas, file_name):
    """
    Resolve o problema TSP com Drone usando Gurobi (MILP).
    """
    start_time = time.time()
    
    # Ajuste: O depósito é o nó 0. O Gurobi precisa que a lista de nós seja consistente.
    # No código original havia um append extra que duplicava o depósito, cuidado com isso.
    # Vamos assumir que 'coordenadas' já vem correta (0 a N-1).
    
    n = len(coordenadas)
    
    # Cria o modelo (Licença deve estar configurada no ambiente)
    try:
        model = gp.Model("TSP_Drone")
    except gp.GurobiError as e:
        print(f"Erro de Licença Gurobi: {e}")
        return

    # Variáveis
    x = model.addVars(n, n, vtype=GRB.BINARY, name="x") # Caminhão
    y = model.addVars(n, n, vtype=GRB.BINARY, name="y") # Drone
    u = model.addVars(n, vtype=GRB.CONTINUOUS, name="u") # MTZ (Subtour elimination)

    # Função Objetivo: Minimizar Custo (Tempo/Distância)
    # Custo Caminhão = K * dist | Custo Drone = 2 * dist (Ida e Volta?) ou Fator de velocidade?
    # Baseado no original: Drone é mais rápido (usa distancia pura ou fator?)
    # Original: Caminhão * K, Drone * 2 (assumindo fator de custo)
    
    obj_expr = gp.quicksum(K * distancia(coordenadas[i], coordenadas[j]) * x[i,j] 
                           for i in range(n) for j in range(n) if i != j) + \
               gp.quicksum(2 * distancia(coordenadas[i], coordenadas[j]) * y[i,j] 
                           for i in range(n) for j in range(n) if i != j)
    
    model.setObjective(obj_expr, GRB.MINIMIZE)

    # --- Restrições (Baseadas no PDF e Código Original) ---

    # 1. Todo cliente (exceto depósito) deve ser visitado exatamente uma vez (Caminhão OU Drone)
    for j in range(1, n):
        model.addConstr(
            gp.quicksum(x[i,j] for i in range(n) if i != j) + 
            gp.quicksum(y[i,j] for i in range(1, n) if i != j) == 1, 
            name=f"visit_{j}"
        )

    # 2. Fluxo do Caminhão: Se entra em k, tem que sair de k
    for k in range(n):
        model.addConstr(
            gp.quicksum(x[i,k] for i in range(n) if i != k) == 
            gp.quicksum(x[k,j] for j in range(n) if j != k),
            name=f"flow_truck_{k}"
        )

    # 3. O Drone só pode sair de um nó onde o caminhão esteve (Sincronia)
    # Simplificação: O drone sai de i e entrega em j. O caminhão deve visitar i?
    # No código original: sum(y[k,j]) <= n * sum(x[i,k]). Isso vincula a saída do drone à visita do caminhão.
    for k in range(1, n): # Ignora depósito nessa lógica específica se necessário
        model.addConstr(
            gp.quicksum(y[k,j] for j in range(n) if j != k) <= 
            n * gp.quicksum(x[i,k] for i in range(n) if i != k),
            name=f"drone_launch_{k}"
        )

    # 4. Restrição de Raio do Drone (Pré-processamento)
    for i in range(n):
        for j in range(n):
            if i != j and distancia(coordenadas[i], coordenadas[j]) > R:
                model.addConstr(y[i,j] == 0, name=f"radius_{i}_{j}")

    # 5. Eliminação de Subrotas (MTZ) para o Caminhão
    for i in range(1, n):
        for j in range(1, n):
            if i != j:
                model.addConstr(u[i] - u[j] + n * x[i,j] <= n - 1, name=f"subtour_{i}_{j}")

    # Parâmetros do Solver
    model.setParam(GRB.Param.OutputFlag, 1) # Mostra log
    model.setParam(GRB.Param.TimeLimit, 60) # Limite de 60s para teste

    model.optimize()

    if model.Status == GRB.OPTIMAL:
        print(f"\nSolução Ótima Encontrada: {model.ObjVal:.2f}")
        plot_rota(file_name, n, x, y, coordenadas)
    else:
        print("Solução ótima não encontrada.")

def parse_input(filepath):
    """Lê o arquivo de instância .txt"""
    coordenadas = []
    with open(filepath, 'r') as f:
        # Lê R e K das duas primeiras linhas
        try:
            line1 = f.readline().strip()
            line2 = f.readline().strip()
            if not line1 or not line2: return None, None, []
            
            R = int(line1)
            K = int(line2)
            
            # Lê coordenadas
            for line in f:
                parts = line.strip().split()
                if len(parts) >= 2:
                    coordenadas.append((int(parts[0]), int(parts[1])))
        except ValueError:
            print(f"Erro ao ler arquivo {filepath}")
            return None, None, []
            
    return R, K, coordenadas

if __name__ == "__main__":
    # Caminho para a pasta de dados (ajuste conforme necessário)
    data_folder = os.path.join(os.path.dirname(__file__), '..', 'data')
    
    # Procura arquivos .txt na pasta data
    instance_files = glob.glob(os.path.join(data_folder, "*.txt"))
    
    if not instance_files:
        print(f"Nenhuma instância encontrada em {data_folder}.")
        print("Certifique-se de criar a pasta 'data' e adicionar os arquivos .txt")
    
    for filepath in instance_files:
        filename = os.path.basename(filepath).replace(".txt", "")
        print(f"\n--- Processando {filename} ---")
        
        R, K, coords = parse_input(filepath)
        if coords:
            print(f"Raio: {R}, Fator K: {K}, Nós: {len(coords)}")
            solve_tsp_drone(R, K, coords, filename)
