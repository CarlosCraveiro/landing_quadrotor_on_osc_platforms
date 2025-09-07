# Pouso de um quadrirrotor em plataforma oscilante utilizando estratégia de controle preditivo

Este repositório contém os códigos e simulações do projeto de Iniciação Científica financiado pela [FAPESP](https://bv.fapesp.br/pt/bolsas/222615/pouso-de-um-quadrirrotor-em-plataforma-oscilante-utilizando-estrategia-de-controle-preditivo/), focado no desenvolvimento e avaliação de estratégias de **Controle Preditivo Baseado em Modelo (MPC)** para o pouso de um quadrotor em plataformas oscilantes, típicas de cenários marítimos.

---

## Resumo do projeto

Veículos Aéreos Autônomos Não Tripulados de decolagem e pouso verticais (VANTs-VTOL) são cada vez mais utilizados na automação de tarefas em setores como o portuário e o petrolífero.
Nesses cenários, as aeronaves precisam operar em condições adversas, com **rajadas de vento** e **plataformas de pouso em oscilação**, exigindo novas estratégias de controle para evitar falhas catastróficas.

Este projeto propõe o uso de **MPC** para lidar com essas condições, com foco específico no pouso seguro em plataformas móveis. A validação é realizada por meio de **simulações em Julia**, comparando o desempenho do controlador proposto com soluções existentes na literatura, como o controlador LQR.

## Estrutura do repositório

* `src/` – código da simulação e controladores
* `notebooks/` – notebooks interativos para análise dos resultados
* `Dockerfile` / `docker-compose.yml` – ambiente reprodutível via Docker

---

## Dependências

O ambiente é todo encapsulado em **Docker**, mas você precisa ter o Docker Engine e o Docker Compose instalados.

### Ubuntu / Debian

```bash
sudo apt update
sudo apt install -y docker.io docker-compose
sudo systemctl enable --now docker
```

### Fedora

```bash
sudo dnf install -y docker docker-compose
sudo systemctl enable --now docker
```

## Como rodar

Clone o repositório e execute:

```bash
docker compose build
docker compose up
```

Após a inicialização, copie e cole no navegador o link exibido no terminal, por exemplo:

```
http://127.0.0.1:8888/tree?token=...
```

Isso abrirá o ambiente Jupyter com os notebooks do projeto.

---

## Créditos

Partes deste projeto foram inspiradas ou adaptadas dos notebooks do curso
[Optimal-Control-16-745/lecture-notebooks-2023](https://github.com/Optimal-Control-16-745/lecture-notebooks-2023), licenciados sob [CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/). Foram realizadas modificações e extensões para adequar o código ao escopo deste trabalho.

## Agradecimentos

Este projeto foi desenvolvido com o apoio da Fundação de Amparo à Pesquisa do Estado de São Paulo (FAPESP), por meio do financiamento de uma bolsa de Iniciação Científica.

## Autores
- Bolsista:
    [ *Carlos Henrique Craveiro Aquino Veras* ](https://bv.fapesp.br/pt/pesquisador/736088/carlos-henrique-craveiro-aquino-veras/)
- Orientador:
    [ *Glauco Augusto de Paula Caurin* ](https://orcid.org/0000-0003-0898-1379)
