import numpy as np
from math import pi
import matplotlib.pyplot as plt
from scipy.spatial import Voronoi,voronoi_plot_2d

class Lec:
    @classmethod
    def calculater_LEC(cls,point_cloud):
        # output_image='test.png'
        vor=Voronoi(point_cloud,incremental=False)
        radius,hull=0,[]
        hull_T=Lec.graham_scan(point_cloud) 
        for a in hull_T:
            hull.append(a)
        hull=np.array(hull)
        for i,vertex in enumerate(vor.vertices):
            vertex_t=np.array(vertex)
            in_hull=Lec.is_point_inside_convex_hull(vertex_t,hull)
            if in_hull==True:
                distances=cls.distance_to_point_set(point_cloud,vertex)
                top3_points=cls.top_smallest(distances)
                new_center,new_radius=vertex,cls.dist(vertex,point_cloud[top3_points[0]])
                if new_radius>radius:
                    radius,center=new_radius,new_center
                    # print('Iterating Candidate',i,center,radius)
            else:
                continue
        # print('Largest empty circle Center:',center,'Radius:',radius)
        # cls.save_results_fig(vor,point_cloud,center,radius,output_image)
        return(center,radius)

    @staticmethod
    def graham_scan(point_cloud):
        def cross_product(o,a,b):
            return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])

        def graham_comparator(point):
            angle = np.arctan2(point[1]-p0[1],point[0]-p0[0])
            return(angle)
        if len(point_cloud)<3:
            print('err: non hull')
            return(point_cloud)
        
        p0=min(point_cloud,key=lambda point:(point[1], point[0]))
        sorted_points=sorted(point_cloud,key=graham_comparator)
        hull = [p0,sorted_points[0]]
        for i in range(1,len(sorted_points)):
            while len(hull)>1 and cross_product(hull[-2],hull[-1],sorted_points[i])<=0:
                hull.pop()
            hull.append(sorted_points[i])
        return(hull)
    
    @staticmethod
    def is_point_inside_convex_hull(point,convex_hull):
        x,y=point
        n=len(convex_hull)
        inside=False
        for i in range(n):
            x1,y1=convex_hull[i]
            x2,y2=convex_hull[(i+1)%n]
            if ((y1<=y<y2) or (y2<=y<y1)) and (x<(x2-x1)*(y-y1)/(y2-y1)+x1):
                inside=not inside
        return(inside)

    @classmethod
    def dist(cls,a,b):
        return(np.linalg.norm(a-b))

    # @classmethod
    # def draw_circle(cls,center,radius):
    #     step_size=0.005
    #     circle=np.zeros((int(1/step_size),2))
    #     t_range=np.arange(0,1,step_size)
    #     for i,t in enumerate(t_range):
    #         circle[i]=np.array([center[0]+radius*np.cos(2*pi*t),center[1]+radius*np.sin(2*pi*t)])
    #     return(circle)
    
    @classmethod
    def distance_to_point_set(cls,array_of_points,pivot_point):
        distances=[]
        for point in array_of_points:
            distances.append(cls.dist(point,pivot_point))
        return(distances)
    
    @classmethod
    def top_smallest(cls,list_of_numbers,nb_of_top=3):
        m1_v,m2_v,m3_v=float('inf'),float('inf'),float('inf')
        m1_i,m2_i,m3_i=float('inf'),float('inf'),float('inf')
        for i,x in enumerate(list_of_numbers):
            if x<=m1_v:
                m1_i,m2_i=i,m1_i
                m1_v,m2_v=x,m1_v
            elif x<m2_v:
                m2_v=x
                m2_i=i
            elif x<m3_v:
                m3_v=x
                m3_i=i
        return(m1_i,m2_i,m3_i)
    
    # @classmethod
    # def save_results_fig(cls,vor,point_cloud,center,radius,output_image):
    #     fig=voronoi_plot_2d(vor,show_vertices=False,line_colors='orange',
    #                         line_width=2,line_alpha=0.5,point_size=5)
    #     current=cls.draw_circle(center,radius)
    #     plt.scatter(center[0],center[1],marker='x')
    #     plt.scatter(current[:,0],current[:,1],marker=".",linewidths=0.05)
    #     for i,n in enumerate(point_cloud):
    #         plt.annotate(str(i), (point_cloud[i,0],point_cloud[i,1]))
    #     fig.set_size_inches(10,10)
    #     fig.savefig(output_image, dpi=300)