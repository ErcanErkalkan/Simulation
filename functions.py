import numpy as np
from typing import List, Optional, Tuple
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components

from uav import UAV
from goal import Goal
from ground import Ground


class Matrix:
    """Utility methods for matrix creation and operations using NumPy."""
    @staticmethod
    def zeros(rows: int, cols: int) -> np.ndarray:
        return np.zeros((rows, cols))

    @staticmethod
    def random(rows: int, cols: int, min_val: float, max_val: float) -> np.ndarray:
        return np.random.uniform(min_val, max_val, (rows, cols))

    @staticmethod
    def identity(n: int) -> np.ndarray:
        return np.eye(n)

    @staticmethod
    def product(a: np.ndarray, b: np.ndarray) -> np.ndarray:
        return np.dot(a, b)

    @staticmethod
    def subtract(a: np.ndarray, b: np.ndarray) -> np.ndarray:
        return np.subtract(a, b)

    @staticmethod
    def as_string(matrix: np.ndarray) -> str:
        return np.array2string(matrix, formatter={"float_kind": lambda x: f"{x:8.3f}"})


class Trigonometry:
    """Trigonometric calculations."""
    @staticmethod
    def degrees_to_radians(degrees: float) -> float:
        return np.deg2rad(degrees)

    @staticmethod
    def radians_to_degrees(radians: float) -> float:
        return np.rad2deg(radians)

    @staticmethod
    def normalize_radians_to_degrees(radians: float) -> float:
        return np.rad2deg(radians) % 360.0


class MatrixOperation:
    """
    Sınıf içindeki ana mantık:
    - `adj_matrix`: UAV'ler ve opsiyonel Ground arasında ÖKLİT mesafe matrisi oluşturur.
    - `find_connected_components`: Mesafesi, "etkin iletişim eşiğinden" (connect_thr) küçük veya
      eşit olan kenarları kullanarak bağlantılı bileşenleri bulur.
    - `find_risky_links`: Farklı bileşenler arasında olup "hemen hemen iletişim eşiğinde" (ör. 0.9*thr < d <= thr)
      kalan linkleri döndürür. Böylece bunlar "kopma riski olan" veya "bir adım daha relay eklenirse bağlanabilecek"
      bağlantılar olarak değerlendirilir.
    """

    @staticmethod
    def adj_matrix(uavs: List[UAV], ground: Optional[Ground] = None) -> np.ndarray:
        """
        UAV'lerin (ve varsa Ground'un) pozisyonlarını alıp
        birbirlerine uzaklıklarını hesaplayan kare matris döndürür.
        
        Örneğin `n` UAV varsa ve `ground` da verilmişse matris boyutu (n+1) x (n+1) olur;
        son satır/sütun ground'a aittir.
        """
        positions = np.array([[uav.pos.x, uav.pos.y] for uav in uavs])
        if ground:
            positions = np.vstack((positions, [ground.pos.x, ground.pos.y]))
        # Her bir (i,j) çifti için Öklid mesafesi
        dist_mat = np.linalg.norm(positions[:, None] - positions[None, :], axis=2)
        return dist_mat

    @staticmethod
    def uav_to_goal(uavs: List[UAV], goals: List[Goal]) -> np.ndarray:
        """
        UAV'lerin hedeflere (goal) olan mesafelerini hesaplayan bir matris döndürür.
        (satırlar UAV, sütunlar Goal olacak şekilde)
        """
        uav_positions = np.array([[uav.pos.x, uav.pos.y] for uav in uavs])
        goal_positions = np.array([[goal.pos.x, goal.pos.y] for goal in goals])
        dist_mat = np.linalg.norm(uav_positions[:, None] - goal_positions[None, :], axis=2)
        return dist_mat

    @staticmethod
    def find_connected_components(adj: np.ndarray, comm_thr: float) -> List[List[int]]:
        """
        Mevcut mesafe matrisine (adj) bakarak, <= connect_thr mesafede olan düğümleri
        "kenarla" bağlanmış kabul eder. Bu sayede graph oluşturulup connected components bulunur.

        NOT: Burada 'connect_thr' = comm_thr * 0.9 gibi bir mantık kullanıyoruz; 
        eğer "daha güvenli" (tam threshold'un biraz altı) bir değere göre bileşenleri tespit etmek istiyorsanız.
        
        Tercihen parametreyi ayarlayarak, "tam" threshold'a göre de yapabilirsiniz:
            connect_thr = comm_thr

        Dönüş: Her bir liste bir bileşeni, liste içi indexler de o bileşendeki düğümleri gösterir.
        """
        # Örneğin "güvenli" bir eşik olarak 0.9 * comm_thr kullanalım:
        connect_thr = comm_thr * 0.9

        # "distance <= connect_thr" ise orada bir edge var
        graph = csr_matrix(adj <= connect_thr)
        _, labels = connected_components(csgraph=graph, directed=False)

        # labels[i] = i. düğümün bileşen numarası
        num_components = labels.max() + 1
        components = []
        for comp_id in range(num_components):
            comp_nodes = [i for i, label in enumerate(labels) if label == comp_id]
            components.append(comp_nodes)

        return components

    @staticmethod
    def find_risky_links(
        components: List[List[int]],
        adj: np.ndarray,
        comm_thr: float
    ) -> List[Tuple[int, int]]:
        """
        Farklı bileşenlerin, mesafesi "connect_thr < distance <= comm_thr" aralığında kalan 
        düğüm çiftlerini "risky link" olarak bulup döndürür.

        Böylece, "biraz daha geniş threshold kullanılsa bu iki bileşen birleşebilirdi" 
        dediğimiz sınırdaki bağlantılar saptanabilir.
        
        Örneğin "connect_thr" = 0.9*comm_thr ise:
          - '0.9*comm_thr < distance <= comm_thr' aralığı, "neredeyse bağlı ama tam değil" durumları gösterir.
        
        Yöntem: Her iki bileşeni (comp_a, comp_b) ele alıp,
                o iki bileşeni birleştirebilecek en kısa "risky" kenarı buluyoruz (min_risk).
        """
        connect_thr = comm_thr * 0.9  # Aynı mantığı kullanalım ki tutarlı olsun.

        risky_links = []
        # Tüm farklı bileşen çiftlerini kontrol ediyoruz:
        for i, comp_a in enumerate(components):
            for comp_b in components[i + 1:]:
                min_risk_dist = float('inf')
                min_risk_link = None
                # comp_a içindeki her düğüm ile comp_b içindeki her düğüm arasında bak:
                for a in comp_a:
                    for b in comp_b:
                        dist_ab = adj[a, b]
                        # "tamamen bağlı" olsalardı zaten aynı bileşende olacaklardı (dist <= connect_thr).
                        # Ama dist, connect_thr'dan büyük => yani o an ki grafa göre farklı bileşenlerde.
                        # Yine de dist <= comm_thr ise, "risky" sayıyoruz.
                        if connect_thr < dist_ab <= comm_thr:
                            if dist_ab < min_risk_dist:
                                min_risk_dist = dist_ab
                                min_risk_link = (a, b)
                # O iki bileşeni birleştirebilecek en yakın linki ekliyoruz:
                if min_risk_link:
                    risky_links.append(min_risk_link)

        return risky_links
