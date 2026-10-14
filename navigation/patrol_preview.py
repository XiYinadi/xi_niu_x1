"""Sequential path preview, with cancellation generations and no actuator API."""
import copy


class PatrolPreview:
    def __init__(self):
        self.generation = 0
        self.state = dict(status='idle', segment=0, total=0, path=[], message='尚未预览巡检路线')

    def cancel(self, reason='巡检路线预览已取消'):
        self.generation += 1
        if self.state['status'] == 'planning':
            self.state.update(status='cancelled', message=reason)

    def snapshot(self):
        return copy.deepcopy(self.state)

    def clear(self):
        self.generation += 1
        self.state = dict(status='idle', segment=0, total=0, path=[], message='尚未预览巡检路线')

    def start(self, task, origin, plan):
        if self.state['status'] == 'planning':
            raise ValueError('巡检路线正在预览，请先取消')
        self.generation += 1
        generation = self.generation
        points = copy.deepcopy(task['waypoints'])
        segments = list(zip([copy.deepcopy(origin)]+points[:-1], points))
        if task['cycles'] > 1:
            segments.append((points[-1], points[0]))
        self.state = dict(status='planning', segment=0, total=len(segments), path=[],
            message='正在计算巡检路线，未驱动车轮',
            planned_stopped_s=sum(p['dwell_s'] for p in points)*task['cycles'])

        def step(index):
            if generation != self.generation:
                return
            if index == len(segments):
                self.state.update(status='ready', message='巡检路线预览完成，未执行巡检')
                return
            self.state.update(segment=index+1, message=f'正在预览第 {index+1}/{len(segments)} 段，未驱动车轮')
            def completed(path, error):
                if generation != self.generation:
                    return
                if error or not path:
                    self.state.update(status='failed', message=f'第 {index+1} 段不可达：'+(error or '规划器返回空路径'))
                    return
                self.state['path'].extend(copy.deepcopy(path[1:] if self.state['path'] else path))
                step(index+1)
            try:
                plan(*segments[index], completed)
            except Exception as e:
                completed(None, str(e))
        step(0)
